//
// Created by 16182 on 10/3/2021.
//

#ifndef SILSIM_SD_H
#define SILSIM_SD_H

#include "Arduino.h"
#include<cstdint>
#include<unordered_map>
#include<string>
#include<vector>

struct FileStorage {
    FileStorage(uint8_t mode) {
        flags = mode;
    };
    std::vector<char> vect;
    uint8_t flags;
};
struct File {
    File();
    File(FileStorage* storage);
    void write(const uint8_t * data, size_t size);
    void flush();
    size_t println(const char * s);
    FileStorage* storage_;
};



#define O_CREAT 0x1
#define O_WRITE 0x2
#define O_TRUNC 0x4

#define BUILTIN_SDCARD 254

struct SDClass{
    bool exists(const char * fileName);
    bool begin(uint8_t csPin = (uint8_t)10);
    File open(const char * file_path, uint8_t mode);
    std::unordered_map<std::string, FileStorage> map;
};

extern SDClass SD;

#endif  // SILSIM_SD_H
