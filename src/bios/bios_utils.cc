#include <bios/bios_utils.hh>
#include <cstring>
#include <iostream>
#include <sstream>
#include <algorithm>
#include <openssl/evp.h>
#include <iomanip>
#include <log/log.hh>
#include <unordered_map>
#include <functional>

const std::unordered_map<std::string, std::function<void(Bus &)>> bios_quirks = {
    // DESR 5000 1.80 BIOS
    {"1c6cd089e6c83da618fbf2a081eb4888",
     [](Bus &bus) {
         bus.dmac.D_ENABLER = 0x1201;
         bus.ram.resize(1024 * 1024 * 64); // Resize to 64MB
         std::fill(bus.ram.begin(), bus.ram.end(), 0);
     }},
    // SCPH-10000 BIOS
    {"acf4730ceb38ac9d8c7d8e21f2614600", [](Bus &bus) {}},
};

std::string compute_md5(const std::vector<uint8_t> &data)
{
    const EVP_MD *md5 = EVP_md5();
    unsigned char md5_result[EVP_MAX_MD_SIZE];
    unsigned int md5_length = 0;

    EVP_MD_CTX *ctx = EVP_MD_CTX_new();
    if (!ctx)
    {
        Logger::error("Failed to create EVP_MD_CTX for MD5 computation");
        return "";
    }

    if (EVP_DigestInit_ex(ctx, md5, nullptr) != 1 || EVP_DigestUpdate(ctx, data.data(), data.size()) != 1 ||
        EVP_DigestFinal_ex(ctx, md5_result, &md5_length) != 1)
    {
        Logger::error("Failed to compute MD5 checksum using EVP API");
        EVP_MD_CTX_free(ctx);
        return "";
    }

    EVP_MD_CTX_free(ctx);

    std::ostringstream md5_string;
    for (unsigned int i = 0; i < md5_length; ++i)
    {
        md5_string << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(md5_result[i]);
    }
    return md5_string.str();
}

void apply_bios_quirks(Bus &bus, const std::vector<uint8_t> &bios)
{
    std::string md5_checksum = compute_md5(bios);
    Logger::info("BIOS MD5 checksum: " + md5_checksum);

    auto it = bios_quirks.find(md5_checksum);
    if (it != bios_quirks.end())
    {
        Logger::info("Applying quirks for BIOS MD5 checksum: " + md5_checksum);
        it->second(bus);
    }
    else
    {
        Logger::warn("Unknown BIOS MD5 checksum: " + md5_checksum);
    }
}

std::vector<romdir_entry> parse_bios(const std::vector<uint8_t> &bios_data, size_t &romdir_offset)
{
    std::vector<romdir_entry> entries;
    const char *reset_str = "RESET";
    size_t reset_len = std::strlen(reset_str);

    // Use a lambda to compare uint8_t to char.
    auto it = std::search(bios_data.begin(), bios_data.end(), reset_str, reset_str + reset_len,
                          [](uint8_t byte, char c) { return static_cast<char>(byte) == c; });

    if (it == bios_data.end())
    {
        std::cerr << "Error: ROMDIR not found (\"RESET\" string not located)." << std::endl;
        return entries;
    }

    romdir_offset = std::distance(bios_data.begin(), it);

    size_t offset = romdir_offset;
    // Iterate over the BIOS data to extract ROMDIR entries.
    // We assume the ROMDIR table is contiguous and terminated by an entry with an empty name.
    while (offset + sizeof(romdir_entry) <= bios_data.size())
    {
        romdir_entry entry;
        std::memcpy(&entry, &bios_data[offset], sizeof(romdir_entry));
        // Terminate when we encounter an entry whose name is all zeros (or at least first byte is 0)
        if (entry.name[0] == '\0')
            break;
        entries.push_back(entry);
        offset += sizeof(romdir_entry);
    }

    return entries;
}

std::vector<uint8_t> extract_file_from_bios(const std::vector<uint8_t> &bios_data, const std::string &file_name)
{
    // Parse the ROMDIR index.
    size_t romdir_offset = 0;
    std::vector<romdir_entry> entries = parse_bios(bios_data, romdir_offset);
    if (entries.empty())
    {
        std::cerr << "Error: No valid ROMDIR entries found." << std::endl;
        return {};
    }

    // For the file layout, the starting offset of file data is computed by summing the
    // file_size fields of all entries that come before the target file. Here we assume
    // that the files are stored consecutively starting at offset 0.
    size_t file_offset = 0;

    std::cout << "Extracting file \"" << file_name << "\"..." << std::endl;

    // Iterate through the ROMDIR entries in order.
    for (const auto &entry : entries)
    {
        // Construct an std::string from the entry's name (trimming any padding).
        std::string entry_name(entry.name, strnlen(entry.name, sizeof(entry.name)));
        if (entry_name == file_name)
        {
            if (file_offset + entry.file_size > bios_data.size())
            {
                std::cerr << "Error: File size exceeds BIOS data boundaries." << std::endl;
                return {};
            }

            printf("Found file \"%s\" at offset %zu, size %u\n", entry_name.c_str(), file_offset, entry.file_size);
            // Return a slice of the BIOS data that corresponds to the file.
            return std::vector<uint8_t>(bios_data.begin() + file_offset,
                                        bios_data.begin() + file_offset + entry.file_size);
        }
        // Add the file size of this entry (ignore ext_info_size) to advance to the next file.
        file_offset += (entry.file_size + 0xF) & ~0xF;
    }

    std::cerr << "Error: File \"" << file_name << "\" not found in BIOS." << std::endl;
    return {}; // Target file not found.
}

std::vector<IOPBTCONFEntry> parse_iopbtconf(const std::vector<uint8_t> &iopbtconf_data)
{
    std::vector<IOPBTCONFEntry> entries;
    std::stringstream ss(std::string(iopbtconf_data.begin(), iopbtconf_data.end()));
    std::string line;

    while (std::getline(ss, line))
    {
        if (line.empty() || line[0] == '#')
        {
            continue; // Skip empty lines and comments
        }

        IOPBTCONFEntry entry;
        if (line[0] == '@')
        {
            entry.type = "Start Address";
            entry.value = line.substr(1);
        }
        else if (line[0] == '!')
        {
            size_t space_pos = line.find(' ');
            entry.type = line.substr(1, space_pos - 1);
            entry.value = line.substr(space_pos + 1);
        }
        else
        {
            entry.type = "Module";
            entry.value = line;
        }

        entries.push_back(entry);
    }

    return entries;
}
