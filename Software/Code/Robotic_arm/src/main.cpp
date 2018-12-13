#include <Arduino.h>
#include <driver/uart.h>
#include <driver/uart.h>
#include <soc/io_mux_reg.h>
#include <vector>

static esp_err_t gpio_output_disable(gpio_num_t gpio_num)
{
    if (gpio_num < 32) {
        GPIO.enable_w1tc = (0x1 << gpio_num);
    } else {
        GPIO.enable1_w1tc.data = (0x1 << (gpio_num - 32));
    }

    // Ensure no other output signal is routed via GPIO matrix to this pin
    REG_WRITE(GPIO_FUNC0_OUT_SEL_CFG_REG + (gpio_num * 4),
              SIG_GPIO_OUT_IDX);

    return ESP_OK;
}

namespace lw {

enum class Command {
    SERVO_MOVE_TIME_WRITE = 1,
    SERVO_MOVE_TIME_READ,
    SERVO_MOVE_TIME_WAIT_WRITE = 7,
    SERVO_MOVE_TIME_WAIT_READ,
    SERVO_MOVE_START = 11,
    SERVO_MOVE_STOP,
    SERVO_ID_WRITE,
    SERVO_ID_READ,
    SERVO_ANGLE_OFFSET_ADJUST = 17,
    SERVO_ANGLE_OFFSET_WRITE,
    SERVO_ANGLE_OFFSET_READ,
    SERVO_ANGLE_LIMIT_WRITE,
    SERVO_ANGLE_LIMIT_READ,
    SERVO_VIN_LIMIT_WRITE,
    SERVO_VIN_LIMIT_READ,
    SERVO_TEMP_MAX_LIMIT_WRITE,
    SERVO_TEMP_MAX_LIMIT_READ,
    SERVO_TEMP_READ,
    SERVO_VIN_READ,
    SERVO_POS_READ,
    SERVO_OR_MOTOR_MODE_WRITE,
    SERVO_OR_MOTOR_MODE_READ,
    SERVO_LOAD_OR_UNLOAD_WRITE,
    SERVO_LOAD_OR_UNLOAD_READ,
    SERVO_LED_CTRL_WRITE,
    SERVO_LED_CTRL_READ,
    SERVO_LED_ERROR_WRITE,
    SERVO_LED_ERROR_READ
};

using Id = uint8_t;

struct Packet {
    Packet() = default;

    Packet( const uint8_t* data, int len ) {
        for ( int i = 0; i < len; i++ ) {
            _data.push_back( data[ i ] );
        }
    }

    template < typename... Args >
    Packet( Id id, Command c, Args... data ) {
        _buildHeader();
        _data.push_back( id );
        _data.push_back( 3 );
        _data.push_back( static_cast< uint8_t >( c ) );
        _pushData( data... );
        _data.push_back( _checksum( _data ) );
    }

    static Packet move( Id id, uint16_t position, uint16_t time ) {
        return Packet( id, Command::SERVO_MOVE_TIME_WRITE,
            position && 0XFF, position >> 8,
            time && 0xFF, time >> 8 );
    }

    void _buildHeader() {
        _data.push_back( 0x55 );
        _data.push_back( 0x55 );
    }

    void _pushData() {}

    template < typename... Args >
    void _pushData( uint8_t d, Args... data ) {
        _data.push_back( d );
        _data[ 3 ]++;
        _pushData( data... );
    }

    static uint8_t _checksum( const std::vector< uint8_t >& data,
        int offset = 2, int end_offset = 0 ) {
        uint8_t sum = 0;
        for ( int i = offset; i < data.size() - end_offset; i++ )
            sum += data[ i ];
        return ~sum;
    }

    int size() {
        if ( _data.size() < 4 )
            return -1;
        return _data[ 3 ];
    }

    bool valid() {
        if ( _data.size() < 6 )
            return false;
        uint8_t c = _checksum( _data, 2, 0 );
        if ( c != _data.back() )
            return false;
        if ( size() + 3 != _data.size() )
            return false;
        return true;
    }

    void dump() {
        Serial.print("[");
        bool first = true;
        for ( auto x : _data ) {
            if ( !first )
                Serial.print(", ");
            first = false;
            Serial.print( x, HEX );
        }
        Serial.println("]");
    }

    std::vector< uint8_t > _data;
};

struct Motor;
struct Bus;

struct Bus {
    Bus( uart_port_t uart, gpio_num_t pin ) :
        _uart( uart ),
        _pin( pin )
    {
        uart_config_t uart_config = {
            .baud_rate = 115200,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        };
        ESP_ERROR_CHECK( uart_param_config( _uart, &uart_config ) );
        _switchToTxMode();
        const int uart_buffer_size = (1024 * 2);
        ESP_ERROR_CHECK(uart_driver_install(_uart, uart_buffer_size, uart_buffer_size,
            10, &_uart_queue, 0));
    }

    void _switchToTxMode() {
        gpio_output_disable( _pin );
        ESP_ERROR_CHECK (uart_set_pin( _uart, _pin, UART_PIN_NO_CHANGE,
            UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE ) );
    }

    void _switchToRxMode() {
        ESP_ERROR_CHECK (uart_set_pin( _uart, UART_PIN_NO_CHANGE, _pin,
            UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE ) );
    }

    void send( const std::vector< uint8_t >& data ) {
        _switchToTxMode();
        auto *buffer = reinterpret_cast< const char * >( data.data() );
        uart_tx_chars( _uart, buffer, data.size() );
        ESP_ERROR_CHECK( uart_wait_tx_done( _uart, 100 ) );
    }

    Packet receive( int len ) {
        _switchToRxMode();
        uint8_t buff[ 32 ];
        uart_read_bytes( _uart, buff, len, 100 );
        return Packet( buff, len );
    }

    QueueHandle_t _uart_queue;
    uart_port_t _uart;
    gpio_num_t _pin;
};

struct Servo {

};

} // namespace lw

lw::Bus bus( UART_NUM_1, GPIO_NUM_32 );

void servoIdWrite(int id);

void setup() {
    // put your setup code here, to run once:
    Serial.begin( 115200 );
    Serial.write( "Init" );
    lw::Packet a( 254, lw::Command::SERVO_OR_MOTOR_MODE_WRITE, 1, 0, 1000 & 0xFF, 1000 >> 8);
    bus.send( a._data );

    // servoIdWrite(4);
}

void moveForwardAndBackward() {
    lw::Packet a( 4, lw::Command::SERVO_MOVE_TIME_WRITE, 0, 0, 255, 8 );
    //auto a = lw::Packet::move( lw::Id( 4 ), 0, 10 );
    bus.send( a._data );
    Serial.print( "A: " );
    a.dump();
    bus.receive( 10 ).dump();
    bus.receive( 10 ).dump();
    bus.receive( 10 ).dump();
    bus.receive( 10 ).dump();
    delay( 4000 );
    lw::Packet b( 4, lw::Command::SERVO_MOVE_TIME_WRITE, 0xFF, 100, 255, 0 );
    bus.send( b._data );
    Serial.print( "B:" );
    b.dump();
    delay( 1500 );
}

//Write servo ID
void servoIdWrite(int id) {
    if( id < 0 || id > 253) {
        Serial.print("servoIdWrite - WRONG ID: ");
        Serial.println(id);
        return;
    }    
    lw::Packet a( 254, lw::Command::SERVO_ID_WRITE, id );
    bus.send( a._data );
    Serial.print("servoIdWrite - ID: ");
    Serial.println(id);
}

//Read servo ID
int servoIdRead() {
    lw::Packet a( 254, lw::Command::SERVO_ID_READ);
    bus.send( a._data );
    Serial.print("servoIdRead - ID: ");
    a.dump();
    bus.receive( 20 ).dump();
    return 0;
}

void loop() {
    //servoIdWrite(4);
    //delay(1000);
    // servoIdRead();

    //moveForwardAndBackward();
    auto a = lw::Packet::move( lw::Id( 3 ), 0, 1000 );
    bus.send( a._data );

    delay(1000);

    a = lw::Packet::move( lw::Id( 3 ), 90 / 0.24, 1000 );

    bus.send( a._data );

    delay(1000);
    Serial.println("Moved");
}