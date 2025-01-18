#pragma once
#include <array>
#include <cstdint>
#include <cstddef>
#include <concepts>
namespace connector_common {

template <uint16_t crc_init, uint16_t poly>
struct CRC16Config {
    static constexpr uint16_t CRC_INIT = crc_init;
    static constexpr uint16_t POLY = poly;
};

template <typename crc_config_t>
concept ISCRCConfigable = requires {
    { crc_config_t::CRC_INIT } -> std::convertible_to<uint16_t>;
    { crc_config_t::POLY } -> std::convertible_to<uint16_t>;
};

constexpr uint16_t bit_reverse_u16(uint16_t x) {
    uint16_t result = 0;
    for (int i = 0; i < 16; ++i) {
        uint16_t nowbit = (x >> i) & 1;
        result |= (nowbit << (15 - i));
    }
    return result;
}

template <ISCRCConfigable config_t>
class CRC16 {
   public:
    static uint16_t modbus_calc(const uint8_t* data, uint32_t num_bytes, uint16_t crc_init = init_value) {
        uint16_t crc = crc_init;
        while (num_bytes--) crc = (crc >> 8) ^ table[(crc ^ *data++) & 0xff];
        return crc;
    }
    static uint16_t modbus_calc_one(const uint8_t* data, uint16_t crc_init = init_value) {
        return (crc_init >> 8) ^ table[(crc_init ^ *data++) & 0xff];
    }
    static constexpr uint16_t init_value = config_t::CRC_INIT;
    static constexpr uint16_t Poly = config_t::Poly;
    static constexpr size_t TableSize = 256;
    static constexpr const std::array<uint16_t, TableSize>& get_table() { return table; };

   private:
    static constexpr uint16_t table_index_generate(size_t i) {
        uint16_t crc = 0, c = i;
        for (uint16_t j = 0; j < 8; ++j) {
            if ((crc ^ c) & 0x0001) {
                crc = (crc >> 1) ^ bit_reverse_u16(config_t::POLY);
            } else
                crc = crc >> 1;
            c = c >> 1;
        }
        return crc;
    }
    template <size_t... Indices>
    static constexpr std::array<uint16_t, TableSize> generate_table(std::index_sequence<Indices...>) {
        return {table_index_generate(Indices)...};
    }
    static constexpr auto table = generate_table(std::make_index_sequence<TableSize>{});
};

}  // namespace EasyRobotCommands