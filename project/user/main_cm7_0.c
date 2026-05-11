#include "zf_common_headfile.h"
#define LED1                    (P19_0)                                         // SPI ���� SPI ������ ����궨����д IPS200_TYPE_SPI

char txt[128];
uint8 communication_count = 0; 

int main(void)
{
  clock_init(SYSTEM_CLOCK_250M); 	                       // ʱ�����ü�ϵͳ��ʼ��<��ر���>
  debug_init();                                                   // ���Դ�����Ϣ��ʼ��
  wireless_uart_init();
  servo_init();                                                   // �����ʼ��
  parameter_init();                                              // ��̬���������ʼ��
  imu660rb_init();
  small_driver_uart_init();                                      // �����ʼ��
  pid_init(&gyro_pid, 30.0f, 0.0f, 0.0f, 0, 10000, 1.0f);      // ���ٶ�pid��ʼ��//30
  pid_init(&angle_pid, 17.0f, 0.0f, 0.0f, 0, 10000, 1.0f);   // �Ƕ�pid��ʼ��//17
  pid_init(&speed_pid, 0.01f, 0.0f, 0.0f, 0, 10000, 1.0f);    // �ٶ�pid��ʼ��
  pit_ms_init(PIT_CH0,1);                                      // ���ٶ��жϣ�����1ms
  
   while(true)
    { 
//       sprintf(txt,"gyro_y|output|angle|output:%f, %f,%f,%f\r\n",imu_data.gyro_y,gyro_pid.output ,pitch_filter.filtering_angle,angle_pid.output);
       sprintf(txt,"t_speed|r_speed: %d, %d,%f\r\n",10,(small_driver_value.receive_left_speed_data + small_driver_value.receive_right_speed_data) / 2,-speed_pid.output);
       wireless_uart_send_string(txt);
       system_delay_ms(20);
    }
}
