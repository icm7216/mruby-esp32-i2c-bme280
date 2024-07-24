#
#   BME280 (I2C) library for mruby-esp32.
#
#   This libraries are adapted from :bme280.rb => lukasjapan/i2c-bme280.
#   https://github.com/lukasjapan/i2c-bme280
#
#   refer to BST-BME280-DS001-24(bst-bme280-ds002.pdf)
#   https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme280-ds002.pdf
#

module SENSOR
  class BME280

    # I2C Bus address = 0x77 (SD0 connect to VCC)
    def initialize(i2c, addr=0x77, options={})
      @i2c = i2c
      @addr = addr
    end

    # after initializing the sensor, trimming parameter is read out
    def init
      # BME280 register setting
      set_register

      # read trimming parameter
      read_trim_params
      self
    end

    # returns all sensor values in a hash
    # t: temperature
    # p: pressure
    # h: humidity
    # @return [Hash] All sensor values
    def all
      data
    end

    # @return [Float] The temperature in Celsius
    def temperature
      data[:t]
    end

    # @return [Float] The pressure in hectoPascal
    def pressure
      data[:p]
    end

    # @return [Float] The humidity in percent (0.00-100.00)
    def humidity
      data[:h]
    end

  private



    def set_register
      # IIR filter coefficient
      #   filter off    =>  0b000
      #   filter  x2    =>  0b001
      #   filter  X4    =>  0b010
      #   filter  x8    =>  0b011
      #   filter x16    =>  0b100
      #
      # oversampling (Temperature, Humidity, Pressure)
      #   sampling none =>  0b000
      #   sampling   x1 =>  0b001
      #   sampling   X2 =>  0b010
      #   sampling   x4 =>  0b011
      #   sampling   x8 =>  0b100
      #   sampling  x16 =>  0b101
      #
      # sensor mode
      #   sleep         => 0b00
      #   forced        => 0b01
      #   normal        => 0b11
      #
      # standby time [ms]
      #       0.5       => 0b000
      #      62.5       => 0b001
      #     125         => 0b010
      #     250         => 0b011
      #     500         => 0b100
      #    1000         => 0b101
      #      10         => 0b110
      #      20         => 0b111
      #

      # set config registers
      config_reg  = 0xF5        # Register address for config settings
      t_sb        = 0b000       # Standby time = 0.5ms (0b000)
      filter      = 0b000       # filter coefficient = OFF
      spi3w_en    = 0           # Disable SPI
      config_val  = (t_sb << 5) | (filter << 2) | spi3w_en
      write(config_reg, config_val)

      # set ctrl_meas registers
      tp_reg      = 0xF4        # Register address for temperature/pressure settings
      osrs_t      = 0b101       # Temperature oversampling = x16 (0b101)
      osrs_p      = 0b101       # Pressure oversampling = x16 (0b101)
      mode        = 0b11        # Normal mode (0b11)
      tp_val      = (osrs_t << 5) | (osrs_p << 2) | mode
      write(tp_reg, tp_val)

      # set ctrl_hum registers
      hum_reg     = 0xF2        # Register address for humidity settings
      osrs_h      = 0b101       # Humidity oversampling = x16 (0b101)
      hum_val     = osrs_h
      write(hum_reg, hum_val)
    end

    # uint16 to int16
    def int16(msb, lsb)
      n = msb * 256 + lsb
      n -= 0x10000 if n >= 0x8000
      n
    end

    # uint8 to int8
    def int8(n)
      n -= 0x100 if n >= 0x80
      n
    end

    # Read compensation parameters of the BME280
    # refer to bst-bme280-ds002.pdf (Page 24)
    def read_trim_params
      # compensation parameter register mapping
      Calibration = Struct.new(
                  # Register Address  Register content      Data type
        :dig_T1,  # 0x88 / 0x89       dig_T1 [7:0] / [15:8] unsigned short
        :dig_T2,  # 0x8A / 0x8B       dig_T2 [7:0] / [15:8] signed short
        :dig_T3,  # 0x8C / 0x8D       dig_T3 [7:0] / [15:8] signed short
        :dig_P1,  # 0x8E / 0x8F       dig_P1 [7:0] / [15:8] unsigned short
        :dig_P2,  # 0x90 / 0x91       dig_P2 [7:0] / [15:8] signed short
        :dig_P3,  # 0x92 / 0x93       dig_P3 [7:0] / [15:8] signed short
        :dig_P4,  # 0x94 / 0x95       dig_P4 [7:0] / [15:8] signed short
        :dig_P5,  # 0x96 / 0x97       dig_P5 [7:0] / [15:8] signed short
        :dig_P6,  # 0x98 / 0x99       dig_P6 [7:0] / [15:8] signed short
        :dig_P7,  # 0x9A / 0x9B       dig_P7 [7:0] / [15:8] signed short
        :dig_P8,  # 0x9C / 0x9D       dig_P8 [7:0] / [15:8] signed short
        :dig_P9,  # 0x9E / 0x9F       dig_P9 [7:0] / [15:8] signed short
        :dig_H1,  # 0xA1              dig_H1 [7:0]          unsigned char
        :dig_H2,  # 0xE1 / 0xE2       dig_H2 [7:0] / [15:8] signed short
        :dig_H3,  # 0xE3              dig_H3 [7:0]          unsigned char
        :dig_H4,  # 0xE4 / 0xE5[3:0]  dig_H4 [11:4] / [3:0] signed short
        :dig_H5,  # 0xE5[7:4] / 0xE6  dig_H5 [3:0] / [11:4] signed short
        :dig_H6,  # 0xE7              dig_H6                signed char
        :t_fine
      )
      calib = []

      # data addresses
      dig_t_reg  = 0x88
      dig_p_reg  = 0x8E
      dig_h_reg1 = 0xA1
      dig_h_reg2 = 0xE1

      data = read(dig_t_reg, 6)
      calib << ((data[1] << 8) | data[0])         # uint16_t dig_T1 [1][0]
      calib << int16(data[3], data[2])            # int16_t  dig_T2 [3][2]
      calib << int16(data[5], data[4])            # int16_t  dig_T3 [5][4]

      data = read(dig_p_reg, 18)
      calib << ((data[1] << 8) | data[0])         # uint16_t dig_P1 [1][0]
      calib << int16(data[3], data[2])            # int16_t  dig_P2 [3][2]
      calib << int16(data[5], data[4])            # int16_t  dig_P3 [5][4]
      calib << int16(data[7], data[6])            # int16_t  dig_P4 [7][6]
      calib << int16(data[9], data[8])            # int16_t  dig_P5 [9][8]
      calib << int16(data[11], data[10])          # int16_t  dig_P6 [11][10]
      calib << int16(data[13], data[12])          # int16_t  dig_P7 [13][12]
      calib << int16(data[15], data[14])          # int16_t  dig_P8 [15][14]
      calib << int16(data[17], data[16])          # int16_t  dig_P9 [17][16]

      data = read(dig_h_reg1, 1)
      calib << data[0]                            # uint8_t  dig_H1 [0]

      data = read(dig_h_reg2, 7)
      calib << int16(data[1], data[0])            # int16_t  dig_H2 [1],[0]
      calib << data[2]                            # uint8_t  dig_H3 [2]

        #  109876543210      bit[11:0]
        #  xxxxxxxx....      dig_H4_msb [11:4]  [3]
        #      ....xxxx      dig_H4_lsb [3:0]   [4]
        #  xxxxxxxxxxxx      dig_H4     [11:0]
        dig_H4_msb = (data[3] >> 4) & 0x0F
        dig_H4_lsb = ((data[3] << 4) & 0xF0) | (data[4] & 0x0F)
      calib << int16(dig_H4_msb, dig_H4_lsb)      # int16_t  dig_H4 [3][4]

        #  109876543210      bit[11:0]
        #  xxxxxxxx....      dig_H5_msb [11:4]  [5]
        #          xxxx....  dig_H5_lsb [7:4]   [4]
        #  xxxxxxxxxxxx      dig_H5     [11:0]
        dig_H5_msb = (data[5] >> 4) & 0x0F
        dig_H5_lsb = ((data[5] << 4) & 0xF0) | (data[4] >> 4)
      calib << int16(dig_H5_msb, dig_H5_lsb)      # int16_t  dig_H5 [4][5]

      calib << int8(data[6])                      # int8_t   dig_H6 [6]

      @calib = Calibration.new(*calib)
    end

    # compensate temperature
    # refer to bst-bme280-ds002.pdf (page: 49)
    def trim_t(adc_T)
      var1 = (adc_T / 16384.0 - @calib[:dig_T1] / 1024.0) * @calib[:dig_T2]
      var2 = ((adc_T / 131072.0 - @calib[:dig_T1] / 8192.0) * (adc_T / 131072.0 - @calib[:dig_T1] / 8192.0)) * @calib[:dig_T3]
      @calib[:t_fine] = var1 + var2
      t = @calib[:t_fine] / 5120.0
    end

    # compensate pressure
    # refer to bst-bme280-ds002.pdf (page: 49)
    def trim_p(adc_P)
      var1 = (@calib[:t_fine] / 2.0) - 64000.0
      var2 = var1 * var1 / 32768.0 * @calib[:dig_P6]
      var2 = var2 + ((var1 * @calib[:dig_P5]) * 2.0)
      var2 = (var2 / 4.0) + (@calib[:dig_P4] * 65536.0)
      var1 = (@calib[:dig_P3] * var1 * var1 / 524288.0 + @calib[:dig_P2] * var1) / 524288.0
      var1 = (1.0 + var1 / 32768.0) * @calib[:dig_P1]

      if var1 == 0
        p = 0.0
      else
        p = 1048576.0 - adc_P
        p = (p - (var2 / 4096.0)) * 6250.0 / var1
        var1 = @calib[:dig_P9] * p * p / 2147483648.0
        var2 = p * @calib[:dig_P8] / 32768.0
        p = p + (var1 + var2 + @calib[:dig_P7]) / 16.0
        p /= 100.0
      end
    end

    # compensate humidity
    # refer to bst-bme280-ds002.pdf (page: 49)
    def trim_h(adc_H)
      h = @calib[:t_fine] - 76800.0
      h1 = adc_H - (@calib[:dig_H4] * 64.0 + @calib[:dig_H5] / 16384.0 * h)
      h2 = @calib[:dig_H2] / 65536.0 * (1.0 + @calib[:dig_H6] / 67108864.0 * h * (1.0 + @calib[:dig_H3] / 67108864.0 * h))
      h = h1 * h2
      h = h * (1.0 - @calib[:dig_H1] * h / 524288.0)
      h = 100.0 if h > 100.0
      h = 0.0 if h < 0.0
      h
    end

    # Measurement of temperature, pressure and humidity
    # return all compensated values
    def data
      # read ADC data
      data_reg = 0xF7
      data = read(data_reg, 8)
      adc_T = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)
      adc_P = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
      adc_H = (data[6] << 8)  |  data[7]

      # Apply compensation to received ADC data and return as hash
      {
        t: trim_t(adc_T),
        p: trim_p(adc_P),
        h: trim_h(adc_H)
      }
    end

    # write to device
    def write(reg_address, data)
      buff = [reg_address, data].pack("C*")
      @i2c.send(buff, @addr)
    end

    # read from device
    def read(reg_address, size = 1)
      receive(reg_address, size, @addr)
    end

  end
end
