#include "wifi_board.h"
#include "codecs/no_audio_codec.h"
#include "display/lcd_display.h"
#include "system_reset.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "mcp_server.h"
#include "led/single_led.h"

#include <esp_log.h>
#include <driver/i2c_master.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <driver/spi_common.h>
#include <vector>
#include <cstdio>

#define TAG "ESP32-S3-LCD-BME280"

class Bme280Sensor {
private:
    i2c_master_dev_handle_t dev_handle_ = nullptr;
    bool initialized_ = false;
    uint16_t dig_T1_;
    int16_t dig_T2_, dig_T3_;
    uint16_t dig_P1_;
    int16_t dig_P2_, dig_P3_, dig_P4_, dig_P5_, dig_P6_, dig_P7_, dig_P8_, dig_P9_;
    uint8_t dig_H1_, dig_H3_;
    int16_t dig_H2_, dig_H4_, dig_H5_;
    int8_t dig_H6_;

    esp_err_t ReadRegs(uint8_t reg, uint8_t* data, size_t len) {
        return i2c_master_transmit_receive(dev_handle_, &reg, 1, data, len, 100);
    }

    esp_err_t WriteReg(uint8_t reg, uint8_t val) {
        std::vector<uint8_t> buf = {reg, val};
        return i2c_master_transmit(dev_handle_, buf.data(), 2, 100);
    }

public:
    bool Init(i2c_master_bus_handle_t bus) {
        std::vector<uint8_t> addrs = {0x76, 0x77};
        for (uint8_t addr : addrs) {
            i2c_device_config_t dev_cfg = {};
            dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
            dev_cfg.device_address = addr;
            dev_cfg.scl_speed_hz = 100000;
            if (i2c_master_bus_add_device(bus, &dev_cfg, &dev_handle_) == ESP_OK) {
                uint8_t id = 0;
                if (ReadRegs(0xD0, &id, 1) == ESP_OK && (id == 0x60 || id == 0x58)) {
                    ESP_LOGI(TAG, "BME280/BMP280 найден по адресу 0x%02X (ID=0x%02X)", addr, id);
                    std::vector<uint8_t> c1(26, 0);
                    std::vector<uint8_t> c2(7, 0);
                    ReadRegs(0x88, c1.data(), 26);
                    ReadRegs(0xE1, c2.data(), 7);
                    dig_T1_ = (c1.at(1) << 8) | c1.at(0);
                    dig_T2_ = (c1.at(3) << 8) | c1.at(2);
                    dig_T3_ = (c1.at(5) << 8) | c1.at(4);
                    dig_P1_ = (c1.at(7) << 8) | c1.at(6);
                    dig_P2_ = (c1.at(9) << 8) | c1.at(8);
                    dig_P3_ = (c1.at(11) << 8) | c1.at(10);
                    dig_P4_ = (c1.at(13) << 8) | c1.at(12);
                    dig_P5_ = (c1.at(15) << 8) | c1.at(14);
                    dig_P6_ = (c1.at(17) << 8) | c1.at(16);
                    dig_P7_ = (c1.at(19) << 8) | c1.at(18);
                    dig_P8_ = (c1.at(21) << 8) | c1.at(20);
                    dig_P9_ = (c1.at(23) << 8) | c1.at(22);
                    dig_H1_ = c1.at(25);
                    dig_H2_ = (c2.at(1) << 8) | c2.at(0);
                    dig_H3_ = c2.at(2);
                    dig_H4_ = (c2.at(3) << 4) | (c2.at(4) & 0x0F);
                    dig_H5_ = (c2.at(5) << 4) | (c2.at(4) >> 4);
                    dig_H6_ = (int8_t)c2.at(6);

                    WriteReg(0xF2, 0x01);
                    WriteReg(0xF4, 0x27);
                    WriteReg(0xF5, 0xA0);
                    initialized_ = true;
                    return true;
                }
                i2c_master_bus_rm_device(dev_handle_);
                dev_handle_ = nullptr;
            }
        }
        ESP_LOGW(TAG, "Датчик BME280 не обнаружен на шине I2C");
        return false;
    }

    bool Read(float& temp_c, float& hum_pct, float& press_mmhg) {
        if (!initialized_) return false;
        std::vector<uint8_t> d(8, 0);
        if (ReadRegs(0xF7, d.data(), 8) != ESP_OK) return false;

        int32_t adc_P = ((int32_t)d.at(0) << 12) | ((int32_t)d.at(1) << 4) | (d.at(2) >> 4);
        int32_t adc_T = ((int32_t)d.at(3) << 12) | ((int32_t)d.at(4) << 4) | (d.at(5) >> 4);
        int32_t adc_H = ((int32_t)d.at(6) << 8) | d.at(7);

        int32_t var1 = ((((adc_T >> 3) - ((int32_t)dig_T1_ << 1))) * ((int32_t)dig_T2_)) >> 11;
        int32_t var2 = (((((adc_T >> 4) - ((int32_t)dig_T1_)) * ((adc_T >> 4) - ((int32_t)dig_T1_))) >> 12) * ((int32_t)dig_T3_)) >> 14;
        int32_t t_fine = var1 + var2;
        temp_c = ((t_fine * 5 + 128) >> 8) / 100.0f;

        int64_t p_v1 = ((int64_t)t_fine) - 128000;
        int64_t p_v2 = p_v1 * p_v1 * (int64_t)dig_P6_;
        p_v2 = p_v2 + ((p_v1 * (int64_t)dig_P5_) << 17);
        p_v2 = p_v2 + (((int64_t)dig_P4_) << 35);
        p_v1 = ((p_v1 * p_v1 * (int64_t)dig_P3_) >> 8) + ((p_v1 * (int64_t)dig_P2_) << 12);
        p_v1 = (((((int64_t)1) << 47) + p_v1)) * ((int64_t)dig_P1_) >> 33;
        if (p_v1 == 0) {
            press_mmhg = 0;
        } else {
            int64_t p = 1048576 - adc_P;
            p = (((p << 31) - p_v2) * 3125) / p_v1;
            p_v1 = (((int64_t)dig_P9_) * (p >> 13) * (p >> 13)) >> 25;
            p_v2 = (((int64_t)dig_P8_) * p) >> 19;
            p = ((p + p_v1 + p_v2) >> 8) + (((int64_t)dig_P7_) << 4);
            float press_pa = (float)p / 256.0f;
            press_mmhg = press_pa * 0.00750062f;
        }

        int32_t h_v = (t_fine - ((int32_t)76800));
        h_v = (((((adc_H << 14) - (((int32_t)dig_H4_) << 20) - (((int32_t)dig_H5_) * h_v)) + ((int32_t)16384)) >> 15) *
              (((((((h_v * ((int32_t)dig_H6_)) >> 10) * (((h_v * ((int32_t)dig_H3_)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) *
              ((int32_t)dig_H2_) + 8192) >> 14));
        h_v = (h_v - (((((h_v >> 15) * (h_v >> 15)) >> 7) * ((int32_t)dig_H1_)) >> 4));
        h_v = (h_v < 0 ? 0 : h_v);
        h_v = (h_v > 419430400 ? 419430400 : h_v);
        hum_pct = (float)(h_v >> 12) / 1024.0f;
        return true;
    }
};

class CompactWifiBoardLCD : public WifiBoard {
private:
    Button boot_button_;
    LcdDisplay* display_;
    i2c_master_bus_handle_t i2c_bus_ = nullptr;
    Bme280Sensor bme280_;

    void InitializeI2cAndSensors() {
        i2c_master_bus_config_t i2c_bus_cfg = {};
        i2c_bus_cfg.i2c_port = I2C_NUM_0;
        i2c_bus_cfg.sda_io_num = BME280_I2C_SDA_PIN;
        i2c_bus_cfg.scl_io_num = BME280_I2C_SCL_PIN;
        i2c_bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
        i2c_bus_cfg.glitch_ignore_cnt = 7;
        i2c_bus_cfg.flags.enable_internal_pullup = true;
        if (i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus_) == ESP_OK) {
            bme280_.Init(i2c_bus_);
        }

        auto& mcp_server = McpServer::GetInstance();
        mcp_server.AddTool("self.environment.get_sensor_data",
            "Получить текущую температуру (в градусах Цельсия), влажность (в процентах) и атмосферное давление (в мм рт. ст.) с датчика BME280 в комнате.",
            PropertyList(),
            [this](const PropertyList& properties) -> ReturnValue {
                float t = 0, h = 0, p = 0;
                if (bme280_.Read(t, h, p)) {
                    std::string json(128, '\0');
                    int len = snprintf(&json.at(0), json.size(),
                        "{\"temperature_c\": %.1f, \"humidity_percent\": %.1f, \"pressure_mmhg\": %.1f}", t, h, p);
                    json.resize(len);
                    return json;
                }
                return std::string("{\"error\": \"Датчик BME280 не отвечает или не подключен\"}");
            });
    }

    void InitializeSpi() {
        spi_bus_config_t buscfg = {};
        buscfg.mosi_io_num = DISPLAY_MOSI_PIN;
        buscfg.miso_io_num = GPIO_NUM_NC;
        buscfg.sclk_io_num = DISPLAY_CLK_PIN;
        buscfg.quadwp_io_num = GPIO_NUM_NC;
        buscfg.quadhd_io_num = GPIO_NUM_NC;
        buscfg.max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
        ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));
    }

    void InitializeLcdDisplay() {
        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;
        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = DISPLAY_CS_PIN;
        io_config.dc_gpio_num = DISPLAY_DC_PIN;
        io_config.spi_mode = DISPLAY_SPI_MODE;
        io_config.pclk_hz = 20 * 1000 * 1000;
        io_config.trans_queue_depth = 10;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI3_HOST, &io_config, &panel_io));

        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = DISPLAY_RST_PIN;
        panel_config.rgb_ele_order = DISPLAY_RGB_ORDER;
        panel_config.bits_per_pixel = 16;
        ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(panel_io, &panel_config, &panel));

        esp_lcd_panel_reset(panel);
        esp_lcd_panel_init(panel);
        esp_lcd_panel_invert_color(panel, DISPLAY_INVERT_COLOR);
        esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY);
        esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);

        display_ = new SpiLcdDisplay(panel_io, panel,
            DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y,
            DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY);
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting) {
                EnterWifiConfigMode();
                return;
            }
            app.ToggleChatState();
        });
    }

public:
    CompactWifiBoardLCD() : WifiBoard(), boot_button_(BOOT_BUTTON_GPIO) {
        InitializeSpi();
        InitializeLcdDisplay();
        InitializeButtons();
        InitializeI2cAndSensors();
        if (DISPLAY_BACKLIGHT_PIN != GPIO_NUM_NC) {
            GetBacklight()->RestoreBrightness();
        }
    }

    virtual AudioCodec* GetAudioCodec() override {
        #ifdef AUDIO_I2S_METHOD_SIMPLEX
        static NoAudioCodecSimplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_SPK_GPIO_BCLK, AUDIO_I2S_SPK_GPIO_LRCK, AUDIO_I2S_SPK_GPIO_DOUT,
            AUDIO_I2S_MIC_GPIO_SCK, AUDIO_I2S_MIC_GPIO_WS, AUDIO_I2S_MIC_GPIO_DIN);
#else
        static NoAudioCodecDuplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS, AUDIO_I2S_GPIO_DOUT, AUDIO_I2S_GPIO_DIN);
#endif
        return &audio_codec;
    }

    virtual Display* GetDisplay() override {
        return display_;
    }

    virtual Backlight* GetBacklight() override {
        if (DISPLAY_BACKLIGHT_PIN != GPIO_NUM_NC) {
            static PwmBacklight backlight(DISPLAY_BACKLIGHT_PIN, DISPLAY_BACKLIGHT_OUTPUT_INVERT);
            return &backlight;
        }
        return nullptr;
    }
};

DECLARE_BOARD(CompactWifiBoardLCD);
