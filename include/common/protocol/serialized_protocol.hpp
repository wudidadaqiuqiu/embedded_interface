#pragma once
#include <cstdint>
#include <map>
#include <ranges>
#include <functional>

#include "common/concepts.hpp"
#include "common/protocol/crc.hpp"

namespace connector_common {
using protocol_size_t = uint16_t;
using protocol_pack_id = uint8_t;
enum protocol_type_e {
    protocol0 = 0,
};

template <ISCRCConfigable crc_config_t, protocol_type_e type>
struct ProtocolConfig {
    using CRC_CONFIG = crc_config_t;
    static constexpr size_t max_len = 128;
    // static constexpr protocol_pack_id ID = id;
    static constexpr protocol_type_e TYPE = type;
};

template <typename protocol_config_t>
concept ISProtocolConfigable = requires {
    ISCRCConfigable<typename protocol_config_t::CRC_CONFIG>;
    // { protocol_config_t::ID } -> std::convertible_to<protocol_pack_id>;
    { protocol_config_t::TYPE } -> std::convertible_to<protocol_type_e>;
    { protocol_config_t::max_len } -> std::convertible_to<size_t>;
};

template <typename T>
concept Packable = requires(T& a) {
    has_get_structure_data_method<T>;
    has_get_struct_data_method<T>;
    typename T::struct_data_t;
    { T::struct_data_t::ID } -> std::common_with<protocol_pack_id>;
};

inline uint8_t need_escape(uint8_t b) {
    if (b == 0x7d || b == 0x7e || b == 0x7f) return 1;
    return 0;
}
inline uint8_t escape(uint8_t b) {
    return b - 0x7d;
}

inline size_t escape_data(uint8_t* data, size_t len, uint8_t* buffer) {
    size_t i = 0;
    for (uint8_t* p = data; p < data + len; p++, i++) {
        if (!need_escape(*p))
            *(buffer + i) = *p;
        else {
            *(buffer + i) = 0x7f;
            ++i;
            *(buffer + i) = escape(*p);
        }
    }
    return i;
}

template <Packable T, ISProtocolConfigable configT>
class PackGenerator /*requires ISProducer*/ {
   public:
    PackGenerator(const T& src_) : temp(pack_start),
                                   is_ended_(false),
                                   is_escaping(false),
                                   crc16(configT::CRC_CONFIG::CRC_INIT),
                                   index(-2),
                                   src(src_),
                                   structure_data((const uint8_t*)&src_.get_structure_data()),
                                   structure_data_size(sizeof(src_.get_structure_data())),
                                   minmax_len_(structure_data_size + 5) {
        static_assert(configT::TYPE == protocol0);
        // static_assert(structure_data_size);
    }

    PackGenerator& operator++() {
        // std::cout << "++ op: index  :"<< std::setw(5) << index << std::endl;
        ++index;
        if (index == -1) {
            temp = T::struct_data_t::ID;
            // std::cout << "ID field: " << std::setw(5) << "0x" << std::hex << static_cast<int>(temp) << std::dec <<std::endl;
            goto crc_calc;
        } else if (index >= structure_data_size) {
            static int num;
            if (index == structure_data_size) {
                // std::cout << std::hex << "crc16 is: " << crc16 << std::dec << std::endl;
                num = escape_data((uint8_t*)&crc16, 2, temp_crc);
                temp_crc[num++] = pack_end;
            }
            if (--num == -1) is_ended_ = true;
            temp = temp_crc[index - structure_data_size];
            goto end;
        } else {
            if (structure_data[index] == pack_start ||
                structure_data[index] == pack_end ||
                structure_data[index] == pack_escape) {
                enter_escaping();
                goto end;
            }
            if (is_escaping) {
                exit_escaping();
                goto end;
            }
            // normal calc crc
            temp = structure_data[index];
        }
    crc_calc:
        crc16 = CRC16<typename configT::CRC_CONFIG>::modbus_calc_one(&temp, crc16);
    end:
        if (!is_ended_) {
            // std::cout << "*it: 0x" << std::hex <<  static_cast<int>(temp) << std::dec << std::endl;
        }
        return (*this);
    }

    bool is_ended() const {
        return is_ended_;
    }

    protocol_size_t minmax_len() const {
        // std::cout << "minmax len is: " << minmax_len_ << std::endl;
        return minmax_len_;
    }

    uint8_t operator*() const {
        return temp;
    }

    void regenerate_init() {
        temp = pack_start;
        is_ended_ = false;
        is_escaping = false;
        crc16 = configT::CRC_CONFIG::CRC_INIT;
        index = -2;
        minmax_len_ = structure_data_size + 5;
    }

   private:
    static constexpr uint8_t pack_start = 0x7d;
    static constexpr uint8_t pack_end = 0x7e;
    static constexpr uint8_t pack_escape = 0x7f;

    uint8_t temp;
    uint8_t temp_crc[5];
    bool is_ended_;
    bool is_escaping;
    uint16_t crc16;
    int index;
    const T& src;
    const uint8_t* structure_data;
    const protocol_size_t structure_data_size;
    protocol_size_t minmax_len_;

    void enter_escaping() {
        is_escaping = true;
        temp = pack_escape;
        crc16 = CRC16<typename configT::CRC_CONFIG>::modbus_calc_one(&structure_data[index], crc16);
        --index;
        ++minmax_len_;
    }

    void exit_escaping() {
        is_escaping = false;
        temp = structure_data[index] - pack_escape;
    }
};

using whole_pkg_check_func = std::function<bool(protocol_pack_id)>;
using update_pkg_func = std::function<void(protocol_pack_id, const uint8_t*, protocol_size_t)>;
template <ISProtocolConfigable configT>
class Unpacker {
   public:
    Unpacker(const std::map<protocol_pack_id, std::function<void(protocol_pack_id, const uint8_t*, protocol_size_t)>>& update_func_map_,
             const std::map<protocol_pack_id, std::function<bool(protocol_pack_id)>>& wholepkg_check_func_map_)
        : update_func_map(update_func_map_),
          wholepkg_check_func_map(wholepkg_check_func_map_),
          start(buffer),
          end(buffer),
          temp(0),
          state(ptc_init) {
        static_assert(configT::TYPE == protocol0);
    }

    Unpacker()
        : start(buffer),
          end(buffer),
          temp(0),
          state(ptc_init) {
        static_assert(configT::TYPE == protocol0);
    }

    void change_map(const std::map<protocol_pack_id, std::function<void(protocol_pack_id, const uint8_t*, protocol_size_t)>>& update_func_map_,
                    const std::map<protocol_pack_id, std::function<bool(protocol_pack_id)>>& wholepkg_check_func_map_) {
        std::lock_guard<std::mutex> lock(mutex);
        update_func_map = update_func_map_;
        wholepkg_check_func_map = wholepkg_check_func_map_;
        // std::cout << "change map" << std::endl;
        // std::cout  << "update_func_map size: " << update_func_map.size() << std::endl;
        // std::cout << update_func_map << std::endl;
    }

    void unpack(const uint8_t* data, size_t len) {
        std::lock_guard<std::mutex> lock(mutex);
        for (size_t i = 0; i < len; ++i) {
            // std::cout << std::hex << std::uppercase << (int)*(data + i) << std::endl << std::dec;
            switch (*(data + i)) {
                case 0x7d:  // 判断包头
                    // std::cout << "unpacker to the start\n";
                    start = end = buffer;
                    state = ptc_start;
                    continue;
                case 0x7e:  // 判断包尾
                    // std::cout << "unpacker to the end\n";
                    // std::cout << "start != end: " << (start != end) << std::endl;
                    if (start != end && wholepkg_check(*start, start, end - start)) {
                        pack_update(*start, start, end - start);
                    }
                    pack_init();
                    continue;
                case 0x7f:  // 进行转义  在包中间：0x7d -> 0x7f 0x00
                            //    0x7e -> 0x7f 0x01
                            //    0x7f -> 0x7f 0x02
                    temp = 0x7d;
                    continue;
                default:
                    temp += *(data + i);
                    break;
            }
            switch (state) {
                case ptc_start:
                    // std::cout << "ptc start temp is" << (int)temp << std::endl;
                    if (is_key_in_map(temp, wholepkg_check_func_map) && is_key_in_map(temp, update_func_map)) {  // 确认该次的consumer
                        *(end++) = temp;
                        state = ptc_data;
                    } else {
                        pack_init();
                    }
                    break;
                case ptc_data:
                    *(end++) = temp;  // 溢出检测
                    break;
                default:
                    break;
            }
            temp = 0;
        }
    }

   private:
    enum protocol_state_e {
        ptc_init = 0,
        ptc_start,
        ptc_data_len,
        ptc_crc8,
        ptc_cmd,
        ptc_data,
        ptc_crc16,
    };
    std::map<protocol_pack_id, update_pkg_func> update_func_map;
    std::map<protocol_pack_id, whole_pkg_check_func> wholepkg_check_func_map;
    uint8_t buffer[configT::max_len];
    uint8_t* start;
    uint8_t* end;
    uint8_t temp;
    protocol_state_e state;
    mutable std::mutex mutex;

    bool wholepkg_check(protocol_pack_id id, const uint8_t* wholepkg, protocol_size_t len) {
        if ((!is_key_in_map(id, wholepkg_check_func_map)) || wholepkg_check_func_map[id](id) == false) {
            // std::cout << "unpacker unpack id is " << id << std::endl;
            return false;
        }
        // std::cout << "unpacker crc check, len is " << len << std::endl;
        // printArray(wholepkg, len);
        if (len <= 3) return false;
        uint16_t crc_val;
        std::memcpy(&crc_val, wholepkg + len - 2, 2);
        // std::cout << "\ncrc_val is " << crc_val << " " << 
            // CRC16<typename configT::CRC_CONFIG>::modbus_calc(wholepkg, len - 2) << std::endl;
        return CRC16<typename configT::CRC_CONFIG>::modbus_calc(wholepkg, len - 2) == crc_val;
    }

    void pack_update(protocol_pack_id id, const uint8_t* pkg, protocol_size_t len) {
        if (is_key_in_map(id, update_func_map) && len > 3)
            update_func_map[id](id, pkg + 1, len - 3);
    }

    void pack_init() {
        start = end = buffer;
        state = ptc_init;
        temp = 0;
    }

    template <typename T, typename U>
    bool is_key_in_map(const T& key, const U& map) {
        auto v = map.find(key);
        // std::cout << "is_key_in_map, key is " << key << std::endl;
        // std::cout << "map size is " << map.size() << std::endl;
        return (v != map.end());
    }
};
}  // namespace EasyRobotCommands