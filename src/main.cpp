#include "videoStable.h"

//---------------------------------------------------------------------------

int main(int argc, char **argv) {

    if (argc < 3) cout << "\nYOU NEED MORE INPUT ARGS!\n" << endl;

    string inputPath = argv[1];
    string outputPath = argv[2];

    if (argc ==  3) {
        VideoStable stabilizer = VideoStable(inputPath, outputPath);
        stabilizer.run();
    }
    else if (argc == 5) {
        VideoStable stabilizer = VideoStable(inputPath, outputPath, stoi(argv[3]), stoi(argv[4]));
        stabilizer.run();
    }

    return 0;
}