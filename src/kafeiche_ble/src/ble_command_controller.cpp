#include "kafeiche_ble/ble_command_controller.hpp"

// Функция для выполнения команды hcitool и получения RSSI
int get_rssi(const std::string& hci_device) {
    std::string command = "sudo hcitool -i " + hci_device + " rssi";
    FILE* fp = popen(command.c_str(), "r");
    if (!fp) {
        std::cerr << "Ошибка при выполнении hcitool для устройства " << hci_device << std::endl;
        return -100; // Возвращаем слабый сигнал в случае ошибки
    }

    char result[128];
    if (fgets(result, sizeof(result), fp) != nullptr) {
        int rssi = std::atoi(result);
        pclose(fp);
        return rssi;
    }

    pclose(fp);
    return -100; // Если не удалось получить RSSI
}

// Конструктор класса BleCommandController
BleCommandController::BleCommandController() : Node("ble_command_controller") {
    // Инициализация публикации в топик cmd_vel
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("tricycle_controller/reference_unstamped", 10);

    // Установка таймера для вызова callback каждые 100 мс
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&BleCommandController::timer_callback, this)
    );
}

// Callback функция для обработки RSSI и публикации команд движения
void BleCommandController::timer_callback() {
    int rssi0 = get_rssi("hci0");
    int rssi1 = get_rssi("hci1");

    // Формируем сообщение Twist
    auto message = geometry_msgs::msg::Twist();

    if (rssi0 > rssi1) {
        // Если сигнал от hci0 сильнее, поворачиваем влево
        message.angular.z = 0.1;  // Угловая скорость влево
        message.linear.x = 0.1;   // Движение вперед
    }
    else if (rssi1 > rssi0) {
        // Если сигнал от hci1 сильнее, поворачиваем вправо
        message.angular.z = -0.1; // Угловая скорость вправо
        message.linear.x = 0.1;
    }

    else if ((rssi1 == -100) || (rssi0 == -100)) {
        message.angular.z = 0;  
        message.linear.x = 0;  
    }
    
    else {
        // Сигналы равны, двигаемся прямо
        message.angular.z = 0.0;
        message.linear.x = 0.1;
    }

    // Публикуем сообщение
    publisher_->publish(message);

    // Выводим значения RSSI для отладки
    RCLCPP_INFO(this->get_logger(), "RSSI: hci0 = %d, hci1 = %d", rssi0, rssi1);
}

// Главная функция для запуска ноды
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BleCommandController>());
    rclcpp::shutdown();
    return 0;
}
