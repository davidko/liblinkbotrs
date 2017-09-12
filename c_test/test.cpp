#include <iostream>
#include <unistd.h>
#include <linkbotrs.h>
using namespace std;
int main() {
    cout << "Enter test robot ID:\n";
    string id;
    getline(cin, id);
    auto l = linkbotFromSerialId(id.c_str());
    linkbotSetJointSpeeds(l, 0x07, 90, 90, 90);
    linkbotMove(l, 0x07, 90, 90, 90);
    linkbotMoveWait(l, 0x07);
    return 0;
}
