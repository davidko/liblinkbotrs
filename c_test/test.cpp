#include <iostream>
#include <linkbotrs.h>
using namespace std;
int main() {
    cout << "Enter test robot ID:\n";
    string id;
    getline(cin, id);
    auto l = linkbot_new(id.c_str());
    linkbot_move(l, 90, 90, 90);
    return 0;
}
