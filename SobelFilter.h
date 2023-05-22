#ifndef SOBEL_FILTER_H_
#define SOBEL_FILTER_H_
#include <systemc>
#include <cmath>
#include <iomanip>
using namespace sc_core;

#include <tlm>
#include <tlm_utils/simple_target_socket.h>

#include "filter_def.h"

struct SobelFilter : public sc_module {
  tlm_utils::simple_target_socket<SobelFilter> tsock;

  sc_fifo<unsigned char> i_r;
  sc_fifo<unsigned char> i_g;
  sc_fifo<unsigned char> i_b;
  sc_fifo<int> o_result;

  SC_HAS_PROCESS(SobelFilter);

  SobelFilter(sc_module_name n): 
    sc_module(n), 
    tsock("t_skt"), 
    base_offset(0) 
  {
    tsock.register_b_transport(this, &SobelFilter::blocking_transport);
    SC_THREAD(do_filter);
  }

  ~SobelFilter() {
	}

  unsigned int base_offset;

  void do_filter(){
	int x=0;
	int y=0;
	int x_m=0;
	int y_m=0;
	int median_count = 0;
  int median[9];
  int median_bitmap[512][512];
	while(true){
		if(y<512){
			if(x==512){
				y++;
				x=0;
				median_count=0;
        wait(CLOCK_PERIOD, SC_NS);
			}else{
			if(x == 0){
				for(unsigned int i = 0; i < 9; i++){
					unsigned char grey = (i_r.read() + i_g.read() + i_b.read()) / 3;
          median[i] = grey;
          wait(CLOCK_PERIOD, SC_NS);
				}
			} else {
				if(median_count == 0){
					for(unsigned int i = 0; i < 3; i++){
            unsigned char grey = (i_r.read() + i_g.read() + i_b.read()) / 3;
            median[i] = grey;
            wait(CLOCK_PERIOD, SC_NS);
					}	
          median_count = 1;                              
				} else if (median_count == 1){
					for(unsigned int i = 3; i < 6; i++){
            unsigned char grey = (i_r.read() + i_g.read() + i_b.read()) / 3;
            median[i] = grey;
            wait(CLOCK_PERIOD, SC_NS);
					}	
					median_count = 2;
				} else if(median_count == 2){
					for(unsigned int i = 6; i < 9; i++){
            unsigned char grey = (i_r.read() + i_g.read() + i_b.read()) / 3;
            median[i] = grey;
            wait(CLOCK_PERIOD, SC_NS);
					}	
					median_count = 0;
				}
			}          
      for(unsigned int i = 8; i > 0; i--){
        for(unsigned int j = 0; j < i-1; j++){
          if(median[j] > median[j+1]){
            unsigned int tmp = median[j];
            median[j] = median[j+1];
            median[j+1] = tmp;
          }
        }
      }  
			median_bitmap[y][x] = median[4];
			x++;
      wait(CLOCK_PERIOD, SC_NS);
      }
		}
   //mean filter
		if(y>2) {
			unsigned int sum = 0;
			if(x_m==512){
				y_m++;
				x_m=0;
			}
			for (unsigned int v = 0; v < 3; v++) {
				for (unsigned int u = 0; u < 3; u++) {
					int yy = y_m + v - 1;
					int xx = x_m + u - 1;
					if (yy >= 0 && yy < 512 && xx >= 0 && xx < 512) {
						sum += median_bitmap[yy][xx] * mask[v][u];                  
					}   
				}
			}
			unsigned int mean = (sum / 10);

			o_result.write(mean);
			x_m++;
		}
	}
 
  }

  void blocking_transport(tlm::tlm_generic_payload &payload, sc_core::sc_time &delay){
    wait(delay);
    // unsigned char *mask_ptr = payload.get_byte_enable_ptr();
    // auto len = payload.get_data_length();
    tlm::tlm_command cmd = payload.get_command();
    sc_dt::uint64 addr = payload.get_address();
    unsigned char *data_ptr = payload.get_data_ptr();

    addr -= base_offset;

    // cout << (int)data_ptr[0] << endl;
    // cout << (int)data_ptr[1] << endl;
    // cout << (int)data_ptr[2] << endl;
    word buffer;

    switch (cmd) {
      case tlm::TLM_READ_COMMAND:
        // cout << "READ" << endl;
        switch (addr) {
          case SOBEL_FILTER_RESULT_ADDR:
            buffer.uint = o_result.read();
            break;
          default:
            std::cerr << "READ Error! SobelFilter::blocking_transport: address 0x"
                      << std::setfill('0') << std::setw(8) << std::hex << addr
                      << std::dec << " is not valid" << std::endl;
          }
        data_ptr[0] = buffer.uc[0];
        data_ptr[1] = buffer.uc[1];
        data_ptr[2] = buffer.uc[2];
        data_ptr[3] = buffer.uc[3];
        break;
      case tlm::TLM_WRITE_COMMAND:
        // cout << "WRITE" << endl;
        switch (addr) {
          case SOBEL_FILTER_R_ADDR:
            i_r.write(data_ptr[0]);
            i_g.write(data_ptr[1]);
            i_b.write(data_ptr[2]);
            break;
          default:
            std::cerr << "WRITE Error! SobelFilter::blocking_transport: address 0x"
                      << std::setfill('0') << std::setw(8) << std::hex << addr
                      << std::dec << " is not valid" << std::endl;
        }
        break;
      case tlm::TLM_IGNORE_COMMAND:
        payload.set_response_status(tlm::TLM_GENERIC_ERROR_RESPONSE);
        return;
      default:
        payload.set_response_status(tlm::TLM_GENERIC_ERROR_RESPONSE);
        return;
      }
      payload.set_response_status(tlm::TLM_OK_RESPONSE); // Always OK
  }
};
#endif
