#include "json.hpp"
#include <iostream>
#include <fstream>
#include "state.hpp"
using json = nlohmann::json;
using namespace std;

const string filePath = "/usd/save.json";

string get_file_contents(const string &filename) {
  FILE *fp = fopen(filename.c_str(), "rb");
  if(!fp) throw errno;
  string contents;
  fseek(fp, 0, SEEK_END);
  contents.resize(std::ftell(fp));
  rewind(fp);
  fread(&contents[0], 1, contents.size(), fp);
  fclose(fp);
  return contents;
}

void write_file_contents(const string &filename, const string &data) {
  FILE *fp = fopen(filename.c_str(), "wb");
  if(!fp) throw errno;
  fwrite(data.data(), data.size(), 1, fp);
  fclose(fp);
}

json& getState();

void saveState() {
    try {
        write_file_contents(filePath, getState().dump());
        puts(getState().dump().c_str());
    } catch(...) {
        printf("Failed to write data.json. Please check write protection / SD inserted.\n");
    }
}

json* curState;

json& getState() {
    if(!curState) {
        curState = new json();
        try {
            *curState = json::parse(get_file_contents(filePath));
            puts(curState->dump().c_str());
        } catch(...) {
            printf("No SD card data found.\n");
        }
    }
    return *curState;
}
