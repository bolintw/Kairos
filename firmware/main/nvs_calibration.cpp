#include "nvs_calibration.hpp"

#include <cstdio>

#include "nvs.h"

namespace {
constexpr char kNamespace[] = "kairos";
constexpr char kKey[] = "calib";
}  // namespace

bool LoadCalibration(CalibrationData& out)
{
    nvs_handle_t handle;
    if (nvs_open(kNamespace, NVS_READONLY, &handle) != ESP_OK) {
        return false;  // namespace doesn't exist yet — never saved
    }

    CalibrationData loaded;
    size_t size = sizeof(loaded);
    const esp_err_t err = nvs_get_blob(handle, kKey, &loaded, &size);
    nvs_close(handle);

    if (err != ESP_OK || size != sizeof(loaded)) {
        return false;
    }
    out = loaded;
    return true;
}

void SaveCalibration(const CalibrationData& data)
{
    nvs_handle_t handle;
    if (nvs_open(kNamespace, NVS_READWRITE, &handle) != ESP_OK) {
        printf("SaveCalibration: nvs_open failed\n");
        return;
    }
    nvs_set_blob(handle, kKey, &data, sizeof(data));
    nvs_commit(handle);
    nvs_close(handle);
}
