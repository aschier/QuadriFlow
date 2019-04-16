#include "quadriflow/config.hpp"
#include "quadriflow/field-math.hpp"
#include "quadriflow/optimizer.hpp"
#include "quadriflow/parametrizer.hpp"

#include <iostream>
#include "ProgramOptions.hxx"

#ifdef WITH_CUDA
#include <cuda_runtime.h>
#endif

Parametrizer field;

int main(int argc, char** argv) {
    setbuf(stdout, NULL);

#ifdef WITH_CUDA
    cudaFree(0);
#endif
    int t1, t2;
    std::string input_obj, output_obj;
    int faces = -1;

    po::parser cmdline_parser;
    cmdline_parser["help"]
        .abbreviation('h')
        .description("print this help screen")
        .callback([&] { std::cerr << cmdline_parser << '\n'; exit(0); });
    cmdline_parser["input"]
        .abbreviation('i')
        .description("Input filename")
        .type(po::string);
    cmdline_parser["output"]
        .abbreviation('o')
        .description("Output filename")
        .type(po::string);
    cmdline_parser["faces"]
        .abbreviation('f')
        .description("Number of Faces")
        .type(po::i32);
    cmdline_parser["sharp"]
        .abbreviation('s')
        .description("Preserve sharp features");
    cmdline_parser["adaptive"]
        .abbreviation('a')
        .description("Adaptive scale");
    cmdline_parser["mcf"]
        .abbreviation('m')
        .description("Use minimum cost flow");
    cmdline_parser["sat"]
        .abbreviation('S')
        .description("Use aggressive SAT");
    cmdline_parser.parse(argc, argv);

    if (argc == 1) {
        std::cerr << cmdline_parser << '\n';
        exit(1);
    }

    if (cmdline_parser["input"].available()) {
        input_obj = cmdline_parser["input"].get().string.c_str();
    } else {
        std::cerr << "Syntax: " << argv[0] << " -i input.obj -o output.obj" << std::endl;
        exit(1);
    }
    if (cmdline_parser["output"].available()) {
        output_obj = cmdline_parser["output"].get().string.c_str();
    } else {
        std::cerr << "Syntax: " << argv[0] << " -i input.obj -o output.obj" << std::endl;
        exit(1);
    }
    if (cmdline_parser["faces"].available()) {
        faces = cmdline_parser["faces"].get().i32;
    }
    if (cmdline_parser["sharp"].available()) {
        field.flag_preserve_sharp = 1;
    }
    if (cmdline_parser["adaptive"].available()) {
        field.flag_adaptive_scale = 1;
    }
    if (cmdline_parser["mcf"].available()) {
        field.flag_minimum_cost_flow = 1;
    }
    if (cmdline_parser["sat"].available()) {
        field.flag_aggresive_sat = 1;
    }

    printf("%d %s %s\n", faces, input_obj.c_str(), output_obj.c_str());
    if (input_obj.size() >= 1) {
        field.Load(input_obj.c_str());
    } else {
        assert(0);
        // field.Load((std::string(DATA_PATH) + "/fertility.obj").c_str());
    }

    printf("Initializing ...\n");
    field.Initialize(faces);

    printf("Solving for orientation field...\n");
    t1 = GetCurrentTime64();

    Optimizer::optimize_orientations(field.hierarchy);
    field.ComputeOrientationSingularities();
    t2 = GetCurrentTime64();
    printf("Needed %lf seconds\n", (t2 - t1) * 1e-3);

    if (field.flag_adaptive_scale == 1) {
        printf("Estimating Slope...\n");
        t1 = GetCurrentTime64();
        field.EstimateSlope();
        t2 = GetCurrentTime64();
        printf("Needed %lf seconds\n", (t2 - t1) * 1e-3);
    }
    printf("Solving for scale...\n");
    t1 = GetCurrentTime64();
    Optimizer::optimize_scale(field.hierarchy, field.rho, field.flag_adaptive_scale);
    field.flag_adaptive_scale = 1;
    t2 = GetCurrentTime64();
    printf("Needed %lf seconds\n", (t2 - t1) * 1e-3);

    printf("Solving for position field...\n");
    t1 = GetCurrentTime64();
    Optimizer::optimize_positions(field.hierarchy, field.flag_adaptive_scale);

    field.ComputePositionSingularities();
    t2 = GetCurrentTime64();
    printf("Needed %lf seconds\n", (t2 - t1) * 1e-3);
    t1 = GetCurrentTime64();
    printf("Solving for index map...\n");
    field.ComputeIndexMap();
    t2 = GetCurrentTime64();
    printf("Indexmap generation needed %lf seconds\n", (t2 - t1) * 1e-3);
    printf("Writing the file...\n");

    if (output_obj.size() < 1) {
        assert(0);
        // field.OutputMesh((std::string(DATA_PATH) + "/result.obj").c_str());
    } else {
        field.OutputMesh(output_obj.c_str());
    }
    printf("finished.\n");
    // field.LoopFace(2);
    return 0;
}
