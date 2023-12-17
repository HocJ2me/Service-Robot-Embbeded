#include <systemc>
#include <iostream>

using namespace sc_core;

class Robot : public sc_module {
public:
  // Khai báo sự kiện và biến ở đây
  sc_event lineSensorEvent;
  sc_event controlEvent;
  sc_event personDetectionEvent;
  sc_event lowBatteryEvent;

  // Tham số của Robot
  double currentPosition = 0;
  double targetPosition = 0;
  double batteryLevel;
  double startPosition; // Biến mới
  int lineSensorValue; // Biến mới

  SC_HAS_PROCESS(Robot);

  Robot(sc_module_name name) : sc_module(name) {
    SC_THREAD(control);
    SC_THREAD(line_sensor);
    SC_THREAD(person_detection);
    SC_THREAD(check_battery);

    // Khởi tạo vị trí bắt đầu
    startPosition = targetPositions[0]; // Thiết lập vị trí "start"
  }

private:
  // Hàm mô phỏng hành vi của robot dựa trên cảm biến dò line
  void line_sensor() {
    while (true) {
      // Mô phỏng việc dò line
      wait(1, SC_SEC);
      lineSensorValue = simulate_line_sensor(); // Thay thế bằng hàm thực tế của cảm biến dò line

      // Thông báo sự kiện điều khiển để điều chỉnh vị trí của robot
      controlEvent.notify();
    }
  }

  // Hàm mô phỏng điều khiển robot để đạt đến một vị trí mục tiêu
  void control() {
    while (true) {
      wait(controlEvent);

      // Điều chỉnh điều khiển của robot dựa trên giá trị của cảm biến dò line
      adjust_robot_control();

      // Mô phỏng điều khiển robot dựa trên vị trí mục tiêu
      simulate_robot_control();

      // Kiểm tra xem có người đứng trước robot không
      if (simulate_person_detection()) {
        std::cout << "Phát hiện người đứng trước. Dừng robot.\n";
      }

      // Kiểm tra xem robot đã đạt đến vị trí mục tiêu chưa
      if (currentPosition == targetPosition) {
        std::cout << "Robot đã đạt đến vị trí mục tiêu: " << getPositionName(targetPosition) << "\n";

        // Nếu robot ở vị trí "start", chọn một trong các vị trí mục tiêu (a, b, c) ngẫu nhiên
        if (currentPosition == startPosition) {
          targetPosition = targetPositions[rand() % 3 + 2]; // Thiết lập vị trí "a", "b", hoặc "c"

        } else if (currentPosition == chargingPosition && batteryLevel > lowBatteryThreshold) {
          // Nếu robot ở vị trí "sạc" và pin không yếu, đặt vị trí mục tiêu về "start"
          targetPosition = chargingPosition;
        } else {
          // Nếu robot không ở vị trí "start" hoặc "sạc", đặt vị trí mục tiêu về "start"
          targetPosition = startPosition;
        }

        std::cout << "Di chuyển đến vị trí mục tiêu: " << getPositionName(targetPosition) << "\n";
      }
    }
  }

  // Hàm mô phỏng phát hiện người đứng trước
  void person_detection() {
    while (true) {
      wait(personDetectionEvent);

      // Mô phỏng phát hiện người đứng trước
      if (simulate_person_detection() && targetPosition != currentPosition) {
        std::cout << "Phát hiện người đứng trước. Dừng robot.\n";
      }
    }
  }

  // Hàm mô phỏng kiểm tra pin
  void check_battery() {
    while (true) {
      wait(5, SC_SEC); // Mô phỏng kiểm tra pin mỗi 5 giây

      // Mô phỏng kiểm tra mức pin
      batteryLevel = simulate_battery_check(); // Thay thế bằng hàm thực tế kiểm tra pin

      // Kiểm tra xem pin có yếu không
      std::cout << "Mức pin hiện tại: " << batteryLevel << "%\n";
      if (batteryLevel <= lowBatteryThreshold) {
        std::cout << "Mức pin yếu. Bắt đầu quá trình sạc pin.\n";
        targetPosition = chargingPosition; // Đặt vị trí mục tiêu về "sạc"
        controlEvent.notify();
      }
    }
  }

  // Hàm mô phỏng hành vi của cảm biến dò line
  int simulate_line_sensor() {
    // Thay thế bằng logic thực tế của cảm biến dò line
    return rand() % 3 - 1; // Giả sử giá trị của lineSensorValue có thể là -1 (lệch trái), 0 (đi thẳng), 1 (lệch phải)
  }

  // Hàm điều chỉnh điều khiển của robot dựa trên giá trị của cảm biến dò line
  void adjust_robot_control() {
    // Điều chỉnh điều khiển của robot dựa trên giá trị của lineSensorValue
    if (targetPosition == currentPosition){
      
      std::cout << "Robot đứng yên \n";
    } else if (lineSensorValue < 0) {
      std::cout << "Điều chỉnh: Lệch trái -> Đi phải\n";
      // Thêm logic để điều chỉnh điều khiển cho trường hợp lệch trái
    } else if (lineSensorValue > 0) {
      std::cout << "Điều chỉnh: Lệch phải -> Đi trái\n";
      // Thêm logic để điều chỉnh điều khiển cho trường hợp lệch phải
    } else {
      std::cout << "Không cần điều chỉnh: Đi thẳng\n";
      // Thêm logic cho trường hợp đi thẳng
    }
  }

  // Hàm mô phỏng hành vi điều khiển robot
  void simulate_robot_control() {
    // Thay thế bằng logic thực tế điều khiển robot
    // std::cout << "Di chuyển đến vị trí mục tiêu: " << getPositionName(targetPosition) << "\n";
    currentPosition = targetPosition ;
        // std::cout << targetPositions<< "\n";

  }

  // Hàm mô phỏng phát hiện người
  bool simulate_person_detection() {
    // Thay thế bằng logic thực tế phát hiện người
    return (rand() % 2 == 0);
  }

  // Hàm mô phỏng kiểm tra pin
  double simulate_battery_check() {
    // Thay thế bằng logic thực tế kiểm tra pin
    return rand() % 100;
  }

  // Tham số bổ sung
  int targetPositions[5] = {0, 1, 2, 3, 4}; // start, sạc, a, b, c
  double chargingPosition = targetPositions[1]; // Vị trí "sạc"
  double lowBatteryThreshold = 40.0; // Ngưỡng pin yếu

  // Hàm để lấy tên của một vị trí
  std::string getPositionName(double position) {
    if (position == 0) {
      return "start";
    } else if (position == 1) {
      return "Sac";
    } else if (position == 2) {
      return "a";
    } else if (position == 3) {
      return "b";
    } else {
      return "c";
    }
  }
};

int sc_main(int, char*[]) {
  // Khởi tạo
  std::cout << "Mô phỏng Robot\n";

  // Tạo và chạy module Robot
  Robot robot("Robot");
  sc_start(100, SC_SEC);

  return 0;
}
