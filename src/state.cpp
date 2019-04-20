#include "json.hpp"
#include <iostream>
#include <fstream>
#include "state.hpp"
#include "display.hpp"
#include "catOS.hpp"
using json = nlohmann::json;
using namespace std;

string filePath = "";
string oldName = "";
bool latestFileNameWrittenOK = false;

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

//From https://crypto.stackexchange.com/questions/16219/cryptographic-hash-function-for-32-bit-length-input-keys
uint32_t perm32(uint32_t x) {
    int n = 12;
    do // repeat for n from 12 downto 1
        x = ((x>>8)^x)*0x6B+n;
    while( --n!=0 );
    return x;
}

//From https://stackoverflow.com/questions/2351087/what-is-the-best-32bit-hash-function-for-short-strings-tag-names
unsigned int quick_hash(const char *str) {
   unsigned int h;
   unsigned const char *p;

   h = 0;
   for (p = (unsigned const char*)str; *p != '\0'; p++)
      h = 37 * h + *p;
   return h;
}

int getRandomSeed() {
    uint32_t seed = 0;
    //Try to get seed data from SD card
    try {
        seed = quick_hash(get_file_contents("/usd/latest.txt").c_str());
    } catch(...) {

    }
    //Get seed data from the environment
    for(int j = 0; j < 25; j++) {
        for(int i = 1; i < 8; i++) {
            seed ^= pros::ADIAnalogIn('A' + i).get_value() << ((i - 1) * 4);
        }
        seed = perm32(seed);
        seed += pros::battery::get_capacity() * 100;
        seed = perm32(seed);
        seed += pros::battery::get_current();
        seed = perm32(seed);
        seed += pros::battery::get_temperature();
        seed = perm32(seed);
        seed += pros::battery::get_voltage();
        seed = perm32(seed);
        seed += pros::c::controller_get_battery_level(pros::E_CONTROLLER_MASTER);
        seed = perm32(seed);
        seed += pros::c::controller_get_battery_capacity(pros::E_CONTROLLER_MASTER);
        seed = perm32(seed);

        pros::delay(10);
    }
    //XOR with current time
    seed ^= pros::millis();
    return seed;
}

string gen_random(int len) {
    srand(getRandomSeed());
    string s(len, ' ');
    static const char alphanum[] =
        "0123456789"
        "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        "abcdefghijklmnopqrstuvwxyz";

    for (int i = 0; i < len; ++i) {
        s[i] = alphanum[rand() % (sizeof(alphanum) - 1)];
    }
    return s;
}

void gen_file_name() {
    try {
        //First, check for data in latest.txt for an old filename.
        oldName = get_file_contents("/usd/latest.txt");
        auto oldFile = get_file_contents(oldName);
        //Then, make a new name.
        auto newName = "/usd/" + gen_random(8);
        //Write it to latest.txt.
        write_file_contents("/usd/latest.txt", newName);
        latestFileNameWrittenOK = true;
        //Copy old file contents to new file
        write_file_contents(newName, oldFile);
        //Return new name.
        filePath = newName;
    } catch(...) {
        //Otherwise, just generate a random name.
        auto newName = "/usd/" + gen_random(8);
        //Return new name.
        filePath = newName;
    }
}

json& getState();

uint32_t lastSaveTime;

void saveState() {
    try {
        auto &state = getState();
        //Record time-of-save
        state["lifetime"] = pros::millis() + lastSaveTime;
        //Get JSON string
        auto data = state.dump();
        //filePath will is guaranteed to be set after getState() runs.
        write_file_contents(filePath, data);
        //if not latestFileNameWrittenOK, make an attempt to write it.
        if(!latestFileNameWrittenOK) {
            write_file_contents("/usd/latest.txt", filePath);
            latestFileNameWrittenOK = true;
        }
    } catch(...) {
        printf("Failed to write data. Please check write protection / SD inserted.\n");
        //If you use line_set here, you get a segfault
        //because the Elliot object isn't initialized.
        pros::Controller m(pros::E_CONTROLLER_MASTER);
        m.set_text(0, 0, "SD CARD FAILURE");
        m.set_text(1, 0, "CHECK TERMINAL ");
        m.set_text(2, 0, "FOR DATA       ");
        printf("%s", getState().dump().c_str());
    }
}

json* curState;

json& getState() {
    if(!curState) {
        curState = new json();
        try {
            gen_file_name();
            printf("Will save to %s\n", filePath.c_str());
            *curState = json::parse(get_file_contents(filePath));
        } catch(...) {
            printf("No SD card data found.\n");
        }
        if(curState->find("lifetime") == curState->end()) {
            (*curState)["lifetime"] = 0.0;
        }
        if(oldName != "") {
            (*curState)["prevName"] = oldName;
        }
        lastSaveTime = (*curState)["lifetime"].get<double>();
    }
    return *curState;
}
