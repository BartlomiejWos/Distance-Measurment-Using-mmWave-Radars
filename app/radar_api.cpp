#include <pybind11/pybind11.h>
#include "ti-external/rf_api.h"
#include "ti-external/dca_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_NAME_LEN 255
#define SUCCESS_STATUS 0 

void read_rfdc_card_fpga_version()
{
    SINT8 version[MAX_NAME_LEN] = {0};
    memset(version, '\0', MAX_NAME_LEN);

    int status = ReadRFDCCard_FpgaVersion(version);

    if (status != SUCCESS_STATUS) {
        printf("Error: Failed to read FPGA version\n");
        return;
    }

    printf("FPGA Version: %s\n", version);
}

void read_rfdc_card_dll_version()
{
    SINT8 version[MAX_NAME_LEN] = {0};
    memset(version, '\0', MAX_NAME_LEN);

    int status = ReadRFDCCard_DllVersion(version);

    if (status != SUCCESS_STATUS) {
        printf("Error: Failed to read DLL version\n");
        return;
    }

    printf("DLL Version: %s\n", version);
}

#ifdef __cplusplus
}
#endif

namespace py = pybind11;

PYBIND11_MODULE(radar_api, module)
{
    module.doc() = "Python bindings for RF FPGA version API via pybind11";
    module.def("read_rfdc_card_fpga_version", &read_rfdc_card_fpga_version);
    module.def("read_rfdc_card_dll_version", &read_rfdc_card_dll_version);
   
}