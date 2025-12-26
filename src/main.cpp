#include "isobus/hardware_integration/can_hardware_interface.hpp"

#include "isobus/hardware_integration/twai_plugin.hpp"

#include "isobus/isobus/can_general_parameter_group_numbers.hpp"

#include "isobus/isobus/can_network_manager.hpp"

#include "isobus/isobus/can_partnered_control_function.hpp"

#include "isobus/isobus/can_stack_logger.hpp"

#include "isobus/isobus/isobus_virtual_terminal_client.hpp"

#include "isobus/isobus/isobus_virtual_terminal_client_update_helper.hpp"

#include "isobus/isobus/isobus_task_controller_client.hpp"

#include "isobus/utility/iop_file_interface.hpp"



#include "console_logger.cpp"

#include "freertos/FreeRTOS.h"

#include "freertos/task.h"

#include "esp_system.h"

#include "esp_log.h"

#include "esp_task_wdt.h"

#include "driver/twai.h"



extern "C" const std::uint8_t object_pool_start[] asm("_binary_object_pool_iop_start");

extern "C" const std::uint8_t object_pool_end[] asm("_binary_object_pool_iop_end");



static std::shared_ptr<isobus::VirtualTerminalClient> virtualTerminalClient = nullptr;

static std::shared_ptr<isobus::VirtualTerminalClientUpdateHelper> vtUpdateHelper = nullptr;

static std::shared_ptr<isobus::TaskControllerClient> taskControllerClient = nullptr;



// Симуляция состояний опрыскивателя

static bool sectionStates[4] = {false, false, false, false};  // 4 секции

static float currentFlowRate = 0.0f;

static float targetFlowRate = 10.0f;  // Начальное значение



static void twai_alert_task(void *arg) {

    uint32_t alerts = 0;

    while (1) {

        twai_read_alerts(&alerts, pdMS_TO_TICKS(1000));

        if (alerts & TWAI_ALERT_BUS_ERROR) {

            twai_stop();

            vTaskDelay(pdMS_TO_TICKS(100));

            twai_start();

        }

        if (alerts & TWAI_ALERT_BUS_OFF) {

            twai_initiate_recovery();

            vTaskDelay(pdMS_TO_TICKS(1000));

        }

    }

}



// Callback для нажатия кнопок (секции)

static void vt_button_callback(const isobus::VirtualTerminalClient::VTKeyEvent &event) {

    if (event.keyEvent == isobus::VirtualTerminalClient::KeyActivationCode::ButtonPressedOrLatched) {

        uint16_t objectID = event.objectID;

        switch (objectID) {

            case 1001:

                sectionStates[0] = !sectionStates[0];

                virtualTerminalClient->send_change_attribute(1001, 0, static_cast<uint32_t>(sectionStates[0] ? 1 : 0));

                break;

            case 1002:

                sectionStates[1] = !sectionStates[1];

                virtualTerminalClient->send_change_attribute(1002, 0, static_cast<uint32_t>(sectionStates[1] ? 1 : 0));

                break;

            case 1003:

                sectionStates[2] = !sectionStates[2];

                virtualTerminalClient->send_change_attribute(1003, 0, static_cast<uint32_t>(sectionStates[2] ? 1 : 0));

                break;

            case 1004:

                sectionStates[3] = !sectionStates[3];

                virtualTerminalClient->send_change_attribute(1004, 0, static_cast<uint32_t>(sectionStates[3] ? 1 : 0));

                break;

        }

    }

}



// Callback для изменения числового значения (установка расхода через Input Number)

static void vt_change_numeric_callback(const isobus::VirtualTerminalClient::VTChangeNumericValueEvent &event) {

    if (event.objectID == 2002) {  // ID Input Number для целевого расхода

        targetFlowRate = static_cast<float>(event.value) / 100.0f;  // Масштаб 0.01 (настрой под свой пул)

        ESP_LOGI("VT", "Target flow rate set to: %.2f l/min", targetFlowRate);

    }

}



extern "C" void app_main() {

    // WDT с увеличенным таймаутом

    esp_task_wdt_config_t wdt_config = {

        .timeout_ms = 10000,

        .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,

        .trigger_panic = true

    };

    esp_task_wdt_init(&wdt_config);



    twai_general_config_t twaiConfig = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_4, GPIO_NUM_5, TWAI_MODE_NORMAL);

    twaiConfig.rx_queue_len = 128;

    twaiConfig.tx_queue_len = 128;

    twaiConfig.alerts_enabled = TWAI_ALERT_ALL;

    twai_timing_config_t twaiTiming = TWAI_TIMING_CONFIG_250KBITS();

    twai_filter_config_t twaiFilter = TWAI_FILTER_CONFIG_ACCEPT_ALL();



    std::shared_ptr<isobus::CANHardwarePlugin> canDriver = std::make_shared<isobus::TWAIPlugin>(&twaiConfig, &twaiTiming, &twaiFilter);



    isobus::CANStackLogger::set_can_stack_logger_sink(&logger);

    isobus::CANStackLogger::set_log_level(isobus::CANStackLogger::LoggingLevel::Error);



    isobus::CANHardwareInterface::set_number_of_can_channels(1);

    isobus::CANHardwareInterface::assign_can_channel_frame_handler(0, canDriver);



    if (!isobus::CANHardwareInterface::start() || !canDriver->get_is_valid()) {

        return;

    }



    xTaskCreate(twai_alert_task, "twai_alert", 8192, NULL, 5, NULL);



    // NAME ECU

    isobus::NAME TestDeviceNAME(0);

    TestDeviceNAME.set_arbitrary_address_capable(true);

    TestDeviceNAME.set_industry_group(2);

    TestDeviceNAME.set_device_class(4);

    TestDeviceNAME.set_function_code(130);

    TestDeviceNAME.set_identity_number(12345);

    TestDeviceNAME.set_ecu_instance(0);

    TestDeviceNAME.set_function_instance(0);

    TestDeviceNAME.set_device_class_instance(0);

    TestDeviceNAME.set_manufacturer_code(999);



    const isobus::NAMEFilter filterVirtualTerminal(isobus::NAME::NAMEParameters::FunctionCode, static_cast<std::uint8_t>(isobus::NAME::Function::VirtualTerminal));

    const std::vector<isobus::NAMEFilter> vtNameFilters = { filterVirtualTerminal };



    const isobus::NAMEFilter filterTaskController(isobus::NAME::NAMEParameters::FunctionCode, static_cast<std::uint8_t>(isobus::NAME::Function::TaskController));

    const std::vector<isobus::NAMEFilter> tcNameFilters = { filterTaskController };



    auto TestInternalECU = isobus::CANNetworkManager::CANNetwork.create_internal_control_function(TestDeviceNAME, 0, 0x80);



    auto TestPartnerVT = isobus::CANNetworkManager::CANNetwork.create_partnered_control_function(0, vtNameFilters);

    auto TestPartnerTC = isobus::CANNetworkManager::CANNetwork.create_partnered_control_function(0, tcNameFilters);



    virtualTerminalClient = std::make_shared<isobus::VirtualTerminalClient>(TestPartnerVT, TestInternalECU);



    const std::uint8_t *testPool = object_pool_start;

    std::uint32_t poolSize = static_cast<std::uint32_t>(object_pool_end - object_pool_start);

    virtualTerminalClient->set_object_pool(0, testPool, poolSize, "sprayer_vt");



    virtualTerminalClient->initialize(true);



    vtUpdateHelper = std::make_shared<isobus::VirtualTerminalClientUpdateHelper>(virtualTerminalClient);

    vtUpdateHelper->initialize();



    // Регистрация callback'ов

    virtualTerminalClient->get_vt_button_event_dispatcher().add_listener(vt_button_callback);

    virtualTerminalClient->get_vt_change_numeric_value_event_dispatcher().add_listener(vt_change_numeric_callback);



    // TC client (опционально, можно закомментировать если не нужен)

    taskControllerClient = std::make_shared<isobus::TaskControllerClient>(TestPartnerTC, TestInternalECU, TestPartnerVT);

    taskControllerClient->initialize(true);



    while (true) {

        virtualTerminalClient->update();

        if (taskControllerClient) taskControllerClient->update();



        // Симуляция приближения текущего расхода к целевому (замени на реальный датчик)

        if (currentFlowRate != targetFlowRate) {

            if (currentFlowRate < targetFlowRate) currentFlowRate += 0.1f;

            if (currentFlowRate > targetFlowRate) currentFlowRate -= 0.1f;

            if (currentFlowRate > targetFlowRate - 0.1f && currentFlowRate < targetFlowRate + 0.1f) currentFlowRate = targetFlowRate;



            virtualTerminalClient->send_change_numeric_value(2001, static_cast<uint32_t>(currentFlowRate * 100));  // ID 2001 - Output Number текущего расхода

        }



        vTaskDelay(pdMS_TO_TICKS(100));

    }



    virtualTerminalClient->terminate();

    if (taskControllerClient) taskControllerClient->terminate();

    isobus::CANHardwareInterface::stop();

}