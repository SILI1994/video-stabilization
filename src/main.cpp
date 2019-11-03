#include "videoStable.h"

//---------------------------------------------------------------------------

int main(int argc, char **argv) {

    if (argc < 3) cout << "\nYOU NEED MORE INPUT ARGS!\n" << endl;

    string inputPath = argv[1];
    string outputPath = argv[2];

    VideoStable stabilizer = VideoStable(inputPath, outputPath);
    stabilizer.run();
  
  
    return 0;
}