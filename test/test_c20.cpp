#include "common/protocol/serialized_protocol.hpp"
#include "connector/connector.hpp"
#include "emi_functional/auto_restarter.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "robot_msg/msg/c20_recv.hpp"
#include "rclcpp/rclcpp.hpp"

using connector::BaudRate;
using connector::Connector;
using connector::ConnectorSendNode;
using connector::ConnectorSingleRecvNode;
using connector::ConnectorType;
using connector::IdPack;
using connector::TtyFrame;

using connector_common::CRC16Config;
using connector_common::protocol_type_e;
using connector_common::ProtocolConfig;
using connector_common::Unpacker;

using robot_msg::msg::C20Recv;

#pragma pack(1)
struct Gcu2GbcPktT {
	uint8_t sync[2];  // 注释 1
	struct {
		uint8_t trig : 3;	// 注释 2
		uint8_t value : 5;	// 注释 3
	} cmd;
	struct {
		uint8_t : 3;
		int8_t fl_sens : 5;	 // 注释 4
	} aux;
	struct {
		uint8_t : 3;
		uint8_t go_zero : 1;  // 注释 5
		uint8_t wk_mode : 2;  // 注释 6
		uint8_t op_type : 2;  // 注释 7
		int16_t op_value;	  // 注释 8
	} gbc[3];				  // 注释 9
	struct {
		uint8_t : 7;
		uint8_t valid : 1;	// 注释 10
		int16_t angle[3];	// 注释 11
		int16_t accel[3];	// 注释 12
	} uav;					// 注释 13
	struct {
		uint32_t vert_fov1x : 7;   // 注释 14
		uint32_t zoom_value : 24;  // 注释 15
		uint32_t reserved : 1;	   // 注释 16
		float target_angle[2];	   // 注释 17
	} cam;
	uint8_t crc[2];	 //
};
#pragma pack()

#pragma pack(1)
struct Gbc2GcuPkt_t{
	uint8_t sync[2];	   // 注释 19
	uint8_t fw_ver;		   // 注释 20
	uint8_t hw_err;		   // 注释 21
	uint8_t inv_flag : 1;  // 注释 22
	uint8_t gbc_stat : 3;  // 注释 23
	uint8_t tca_flag : 1;  // 注释 24
	uint8_t : 3;
	struct {
		uint8_t stat : 3;	// 注释 25
		uint8_t value : 5;	// 注释 26
	} cmd;
	int16_t cam_rate[3];   // 注释 27
	int16_t cam_angle[3];  // 注释 28
	int16_t mtr_angle[3];  // 注释 29
	uint8_t crc[2];		   // 注释 30
};
#pragma pack()

inline void print_array(const uint8_t* arr, size_t length) {
	std::cout << std::hex;
	for (size_t i = 0; i < length; ++i) {
		std::cout << "0x" << static_cast<int>(arr[i]) << " ";
	}

	std::cout << std::dec;
	if (length != 0 && arr[length - 1] == 0x7e) std::cout << std::endl;
}

static constexpr uint8_t PEER_ID = 0x01;

auto CalculateCrc16(uint8_t* ptr, uint8_t len) -> uint16_t {
	uint16_t crc;
	uint8_t da;
	uint16_t crc_ta[16] = {
		0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
		0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
	};
	crc = 0;
	while (len-- != 0) {
		da = crc >> 12;
		crc <<= 4;
		crc ^= crc_ta[da ^ (*ptr >> 4)];
		da = crc >> 12;
		crc <<= 4;
		crc ^= crc_ta[da ^ (*ptr & 0x0F)];
		ptr++;
	}
	return (crc);
}

class TtyNode : public rclcpp::Node {
   public:
	TtyNode() : Node("test_c20"), connector(), crn(connector), cs(connector) {
		std::string key; 
        this->declare_parameter<std::string>("tty_key", "/dev/ttyUSB0");
		this->get_parameter("tty_key", key);
		LOG_INFO(1, "test_c20: tty_key: %s", key.c_str());

		auto_restarter.set(
			[&]() { return !connector.is_stoped; },
			[&]() { 
				connector.con_open(key, BaudRate::BAUD_115200);},
			[&, key]() { 
				LOG_INFO(1, "restart tty");
				connector.con_close();
				try {
					connector.con_open(key, BaudRate::BAUD_115200);
				} catch (std::exception & e) {
					LOG_ERROR(1, "open tty error: %s", e.what());
					return;
				} 
			}
		);
		auto_restarter.start();
		
		// id_pack.data 长度 设置为 sizeof(Gcu2GbcPktT)
		publisher_ = this->create_publisher<C20Recv>("/c20_recv", 10);
		init_gcu(gcu2gbc_pkt, 0);

		refsubscription = this->create_subscription<geometry_msgs::msg::Twist>(
			"gimbal_ref", 10,
			[this](const geometry_msgs::msg::Twist::SharedPtr msg) {
				// for (int i = 0; i < 2; i++){
				//     ref[i] = msg->data[i];
				// }
				pitch = msg->angular.y;
				yaw = msg->angular.z;
				if (abs(pitch) > 90.0) {
					pitch = 90.0 * (pitch > 0.0 ? 1.0 : -1.0);
				}
				if (abs(yaw) > 90.0) {
					yaw = 90.0 * (yaw > 0.0 ? 1.0 : -1.0);
				}
				std::cout << "pitch: " << pitch << " yaw: " << yaw << std::endl;
			});

		std::map<uint8_t,
				 std::function<void(uint8_t, const uint8_t*, uint16_t)>>
			update_func_map;
		std::map<uint8_t, std::function<bool(uint8_t)>> check_id_func_map;
		check_id_func_map[PEER_ID] = [](uint8_t _) -> bool {
			(void)_;
			std::cout << "check id" << std::endl;
			return true;
		};
		update_func_map[PEER_ID] = [&](uint8_t id, const uint8_t* data,
									   uint16_t length) -> void {
			(void)id;
			(void)data;
			std::cout << "length: " << length << std::endl;
			// if (length > sizeof(imu_data)) return;
			// memcpy(&imu_data, data, length);

			// 1000Hz
			// publisher_->publish(imu_msg);
			
		};
		unpacker.change_map(update_func_map, check_id_func_map);
		crn.register_callback([&](const TtyFrame::MSGT& frame) -> void {
			// std::cout << "recv: ";
			// unpacker.unpack(frame.data.data(), frame.data.size());
			// printArray(frame.data.data(), frame.data.size());
			// RCLCPP_INFO(this->get_logger(), "recv: %s",
			// frame.data.c_str());
			if (frame.data.size() != sizeof(Gbc2GcuPkt_t)) return;
			Gbc2GcuPkt_t* packet = (Gbc2GcuPkt_t*)frame.data.data();
			gbc2gcu_pkt = *packet;
			// 校验协议头和 CRC
			if (gbc2gcu_pkt.sync[0] == 0xB5 && gbc2gcu_pkt.sync[1] == 0x9A) {
				uint16_t crc = CalculateCrc16((uint8_t*)&gbc2gcu_pkt, sizeof(gbc2gcu_pkt) - 2);
				if (gbc2gcu_pkt.crc[0] == (crc>> 8) && gbc2gcu_pkt.crc[1] == (crc& 0xFF)) {
					
				} else {
					return;
				}
			}

			c20_recv_msg.fw_ver = gbc2gcu_pkt.fw_ver;
			c20_recv_msg.hw_err = gbc2gcu_pkt.hw_err;
			c20_recv_msg.inv_flag = gbc2gcu_pkt.inv_flag;
			c20_recv_msg.gbc_stat = gbc2gcu_pkt.gbc_stat;
			c20_recv_msg.tca_flag = gbc2gcu_pkt.tca_flag;
			c20_recv_msg.cmd_stat = gbc2gcu_pkt.cmd.stat;
			c20_recv_msg.cmd_value = gbc2gcu_pkt.cmd.value;
			c20_recv_msg.cam_rate.resize(3);
			for (int i = 0; i < 3; i++) {
				c20_recv_msg.cam_rate[i] = gbc2gcu_pkt.cam_rate[i] * 0.1f;
			}
			// std::swap(c20_recv_msg.cam_angle[0], c20_recv_msg.cam_angle[1]);
			// float tmp = c20_recv_msg.cam_angle[0];
			// c20_recv_msg.cam_angle[0] = c20_recv_msg.cam_angle[1];
			// c20_recv_msg.cam_angle[1] = tmp;
			swap(c20_recv_msg.cam_rate[0], c20_recv_msg.cam_rate[1]);
			c20_recv_msg.cam_angle.resize(3);
			for (int i = 0; i < 3; i++) {
				c20_recv_msg.cam_angle[i] = gbc2gcu_pkt.cam_angle[i] * 0.01f;
			}
			c20_recv_msg.mtr_angle.resize(3);
			for (int i = 0; i < 3; i++) {
				c20_recv_msg.mtr_angle[i] = gbc2gcu_pkt.mtr_angle[i] * 0.01f;
			}
			// tmp = c20_recv_msg.mtr_angle[0];
			// c20_recv_msg.mtr_angle[0] = c20_recv_msg.mtr_angle[1];
			// c20_recv_msg.mtr_angle[1] = tmp;
			swap(c20_recv_msg.mtr_angle[0], c20_recv_msg.mtr_angle[1]);
			// std::cout << "c20 recv msg: " << std::endl;
			publisher_->publish(c20_recv_msg);
		});

		// 定时器
		timer_ = this->create_wall_timer(
			std::chrono::milliseconds(20), [this]() -> void {
				update_gcu(gcu2gbc_pkt);
				// IdPack id_pack;
				id_pack.id = 0;
				id_pack.data.resize(sizeof(gcu2gbc_pkt));
				// copy gcu2gbc_pkt to id_pack.data
				std::memcpy(id_pack.data.data(), &gcu2gbc_pkt,
							sizeof(gcu2gbc_pkt));
				if (!connector.is_stoped)
					cs.send(id_pack);
				// std::cout << "send: " << id_pack.id << std::endl;
			});
	}

	void swap(float& a, float& b) {
		float temp = a;
		a = b;
		b = temp;
	}
	void init_gcu(Gcu2GbcPktT& gcu2gbc_pkt, int8_t sen) {
		gcu2gbc_pkt.sync[0] = 0xA9;
		gcu2gbc_pkt.sync[1] = 0x5B;

		gcu2gbc_pkt.cmd.trig = 0;
		gcu2gbc_pkt.cmd.value = 4;	// 手动控制
		// [-16,15]
		gcu2gbc_pkt.aux.fl_sens = sen;
		gcu2gbc_pkt.gbc[0].go_zero = 0;
		gcu2gbc_pkt.gbc[0].wk_mode = 1;	 // fpv
		gcu2gbc_pkt.gbc[0].op_type = 0;

		gcu2gbc_pkt.gbc[1].go_zero = 0;
		gcu2gbc_pkt.gbc[1].wk_mode = 1;	 // fpv
		gcu2gbc_pkt.gbc[1].op_type = 0;

		gcu2gbc_pkt.gbc[2].go_zero = 0;
		gcu2gbc_pkt.gbc[2].wk_mode = 1;	 //1: 控制云台姿态角 2: fpv
		gcu2gbc_pkt.gbc[2].op_type = 0;

		// 没有载机IMU
		gcu2gbc_pkt.uav.valid = 0;
	}

	void update_gcu(Gcu2GbcPktT& gcu2gbc_pkt) {
		// 0- 滚转；1- 俯仰；2- 偏航
		gcu2gbc_pkt.gbc[0].op_value = 0;
		gcu2gbc_pkt.gbc[1].op_value = (int16_t)(pitch * 100);
		gcu2gbc_pkt.gbc[2].op_value = (int16_t)(yaw * 100);

		// CRC16 校验
		uint16_t crc =
			CalculateCrc16((uint8_t*)&gcu2gbc_pkt, sizeof(gcu2gbc_pkt) - 2);
		gcu2gbc_pkt.crc[0] = (crc >> 8) & 0xFF;
		gcu2gbc_pkt.crc[1] = crc & 0xFF;
		// std ::cout << "crc: " << crc << std::endl;
	}

   private:
	float pitch;
	float yaw;
	IdPack id_pack;
	C20Recv c20_recv_msg;
	Gcu2GbcPktT gcu2gbc_pkt;
	Gbc2GcuPkt_t gbc2gcu_pkt;
	Connector<ConnectorType::TTY> connector;
	ConnectorSingleRecvNode<ConnectorType::TTY, TtyFrame> crn;
	ConnectorSendNode<ConnectorType::TTY, TtyFrame> cs;
	Unpacker<
		ProtocolConfig<CRC16Config<0xFFFF, 0x1021>, protocol_type_e::protocol0>>
		unpacker;
	
	AutoRestarter auto_restarter;
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr refsubscription;

	// rclcpp::Subscription<MotorRef>::SharedPtr subscription_;
	// rclcpp::Subscription<PidParamSet>::SharedPtr
	// subscription_controller_param;
	rclcpp::Publisher<C20Recv>::SharedPtr publisher_;
	rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<TtyNode>());
	rclcpp::shutdown();
	return 0;
}