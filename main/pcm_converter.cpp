#include <fstream>
#include <iostream>

int main(int argc, char** argv) {
    std::ifstream ifs(argv[1]);
    std::cout<<"{"<<std::endl;
    while (true) {
        char c;
        ifs >> c;
        if (ifs.eof())
            break;
        else
            std::cout<<",";
        std::cout<<"0x"<< std::hex<< (0xFF & c);
    }
    std::cout<<"};"<<std::endl;
}