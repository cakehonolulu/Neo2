#pragma once

#include <string>
#include <vector>
#include <unordered_map>
#include <functional>
#include <bus/bus.hh>

struct romdir_entry {
    char name[10];
    uint16_t ext_info_size;
    uint32_t file_size;
};

struct IOPBTCONFEntry
{
    std::string type;
    std::string value;
};

std::vector<romdir_entry> parse_bios(const std::vector<uint8_t> &bios_data, size_t &romdir_offset);
std::vector<uint8_t> extract_file_from_bios(const std::vector<uint8_t> &bios_data, const std::string &file_name);
std::vector<IOPBTCONFEntry> parse_iopbtconf(const std::vector<uint8_t> &iopbtconf_data);
std::string compute_md5(const std::vector<uint8_t> &data);
void apply_bios_quirks(Bus &bus, const std::vector<uint8_t> &bios);