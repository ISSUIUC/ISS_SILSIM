#include <iostream>
#include <string>
#include <fstream>

int main() {
    std::ifstream infile;
    
    infile.open("src/Rocket/values.txt");

    std::string l;
    double d;

    while(!infile.eof()) {
        infile >> l >> d;
        std::cout << l << "\t" << d << std::endl;
    }
}