#include <cstdio>
#include <cstring>
#include <cassert>
#include "ctr.h"
#include "ctr_man.h"

int main(int argc, char * argv[]) {
    char * inFile = NULL, * outFile = NULL;

    // parse args
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--input") == 0) {
            assert(i < argc - 1);
            inFile = argv[++i];
        } else if (strcmp(argv[i], "--output") == 0) {
            assert(i < argc - 1);
            outFile = argv[++i];
        }
    }
    if (inFile == NULL) {
        printf("Input file path not specified!\n");
        return 1;
    }

    CTRMan ctrMan;
    ctrMan.read(inFile);

    ctrMan.cluster();
    // ctrMan.rgmZeroSkew();
    ctrMan.route();


    if (outFile != NULL)
        ctrMan.write(outFile);

    return 0;
}