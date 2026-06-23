#include "eyou.h"
#include "unistd.h"

int main()
{
    // 左臂
    MotorCAN eyou_l("can1");

    eyou_l.bringUpCAN(1000000);        //调试过程中只运行第一次
    uint8_t motor_id_1 = 2;
    uint8_t motor_id_2 = 3;
    uint8_t motor_id_3 = 4;
    uint8_t motor_id_4 = 5;
    uint8_t motor_id_5 = 6;
    uint8_t motor_id_6 = 7;
    bool enable_1 = eyou_l.enableMotor(motor_id_1);
    bool enable_2 = eyou_l.enableMotor(motor_id_2);
    bool enable_3 = eyou_l.enableMotor(motor_id_3);
    bool enable_4 = eyou_l.enableMotor(motor_id_4);
    bool enable_5 = eyou_l.enableMotor(motor_id_5);
    bool enable_6 = eyou_l.enableMotor(motor_id_6);

    if (enable_1 && enable_2 && enable_3 && enable_4 && enable_5 && enable_6) 
    {
        std::cout << "Motor enabled!\n";

        /*单电机位置轮廓模式*/
        // eyou.setProfilePosition(7, 1.57, 2, 2.4, 1.5, 1.5);

        /*多电机位置轮廓模式*/
        std::vector<uint8_t> ids = {2, 3, 4, 5, 6, 7};
        // std::vector<double> positions = {0, 0, 0, 0, 0, 0};                     // 目标角度 (rad)
        std::vector<double> positions = {-0.5, 0.1, 0, -0.5, 0, 0};                     // 目标角度 (rad)

        std::vector<double> speeds = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5};                    // 目标速度 (rad/s)
        std::vector<float> torques = {5.0f, 5.0f, 5.0f, 5.0f, 2.0f, 2.0f};              // 目标力矩 (Nm)
        std::vector<double> accels = {8.0, 8.0, 8.0, 8.0, 8.0, 8.0};                    // 加速度 (rad/s^2)
        std::vector<double> decels = {8.0, 8.0, 8.0, 8.0, 8.0, 8.0};                    // 减速度 (rad/s^2)
        if (eyou_l.setProfilePositionMulti(ids, positions, speeds, torques, accels, decels)) {
            printf("All motors commanded successfully.\n");
        } else {
            printf("Command failed for one or more motors.\n");
        }
        
        /*单电机速度模式*/
        // eyou.setSpeedRad(13,3.14);

        /*单电机力矩模式*/
        // eyou.setTorqueMode(13,1);

        /*单电机调零函数*/
        // eyou.zeroCalibrate(5);

        /*单电机读取位置函数*/
        double pos = eyou_l.readPosition(6);
        std::cout << "position:" << pos << "deg\n";

        /*单电机读取速度函数*/
        double speed = eyou_l.readSpeed(6);
        std::cout << "speed:" << speed << "rad/s\n";

        /*单电机读取力矩（Q值）函数*/
        // double TorqueRatioRaw = eyou.getTorqueRatioRaw(13);
        // std::cout << "TorqueRatioRaw:" << TorqueRatioRaw << "\n"; 

        /*单电机读取力矩函数*/
        double Torque = eyou_l.readTorque(6);
        std::cout << "Torque:" << Torque << "Nm\n";
   
    }
    else
        std::cout << "Motor enable failed!\n";


    //右臂 
    MotorCAN eyou_r("can2");
    
    eyou_r.bringUpCAN(1000000);        //调试过程中只运行第一次
    uint8_t motor_id_7 = 8;
    uint8_t motor_id_8 = 9;
    uint8_t motor_id_9 = 10;
    uint8_t motor_id_10 = 11;
    uint8_t motor_id_11 = 12;
    uint8_t motor_id_12 = 13;
    bool enable_7 = eyou_r.enableMotor(motor_id_7);
    bool enable_8 = eyou_r.enableMotor(motor_id_8);
    bool enable_9 = eyou_r.enableMotor(motor_id_9);
    bool enable_10 = eyou_r.enableMotor(motor_id_10);
    bool enable_11 = eyou_r.enableMotor(motor_id_11);
    bool enable_12 = eyou_r.enableMotor(motor_id_12);

    if (enable_7 && enable_8 && enable_9 && enable_10 && enable_11 && enable_12) 
    {
        std::cout << "Motor enabled!\n";

        /*单电机位置轮廓模式*/
        // eyou.setProfilePosition(7, 1.57, 2, 2.4, 1.5, 1.5);

        /*多电机位置轮廓模式*/
        std::vector<uint8_t> ids = {8, 9, 10, 11, 12, 13};
        // std::vector<double> positions = {0, 0, 0, 0, 0, 0};                     // 目标角度 (rad)
        std::vector<double> positions = {0.5, -0.1, 0, 0.5, 0, 0};                     // 目标角度 (rad)

        std::vector<double> speeds = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5};                    // 目标速度 (rad/s)
        std::vector<float> torques = {5.0f, 5.0f, 5.0f, 5.0f, 2.0f, 2.0f};              // 目标力矩 (Nm)
        std::vector<double> accels = {8.0, 8.0, 8.0, 8.0, 8.0, 8.0};                    // 加速度 (rad/s^2)
        std::vector<double> decels = {8.0, 8.0, 8.0, 8.0, 8.0, 8.0};                    // 减速度 (rad/s^2)
        if (eyou_r.setProfilePositionMulti(ids, positions, speeds, torques, accels, decels)) {
            printf("All motors commanded successfully.\n");
        } else {
            printf("Command failed for one or more motors.\n");
        }
        
        /*单电机速度模式*/
        // eyou.setSpeedRad(13,3.14);

        /*单电机力矩模式*/
        // eyou.setTorqueMode(13,1);

        /*单电机调零函数*/
        // eyou.zeroCalibrate(5);

        /*单电机读取位置函数*/
        double pos = eyou_r.readPosition(8);
        std::cout << "position:" << pos << "deg\n";

        /*单电机读取速度函数*/
        double speed = eyou_r.readSpeed(8);
        std::cout << "speed:" << speed << "rad/s\n";

        /*单电机读取力矩（Q值）函数*/
        // double TorqueRatioRaw = eyou.getTorqueRatioRaw(13);
        // std::cout << "TorqueRatioRaw:" << TorqueRatioRaw << "\n"; 

        /*单电机读取力矩函数*/
        double Torque = eyou_r.readTorque(8);
        std::cout << "Torque:" << Torque << "Nm\n";
   
    }
    else
        std::cout << "Motor enable failed!\n";


    // eyou_r.disableMotor(7);
    // eyou_r.disableMotor(6);
    // eyou_r.disableMotor(5);
    // eyou_r.disableMotor(4);
    // eyou_r.disableMotor(3);
    // eyou_r.disableMotor(2);         

    // eyou_l.disableMotor(7);
    // eyou_l.disableMotor(6);
    // eyou_l.disableMotor(5);
    // eyou_l.disableMotor(4);
    // eyou_l.disableMotor(3);
    // eyou_l.disableMotor(2);
    // eyou_l.bringDownCAN(); 
    // eyou_r.bringDownCAN();             //调试过程中最后一次再运行
    return 0;
}