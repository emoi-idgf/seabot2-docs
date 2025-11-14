
.. _program_listing_file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_driver_gpsd_client_src_gpsd_node.cpp:

Program Listing for File gpsd_node.cpp
======================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_driver_gpsd_client_src_gpsd_node.cpp>` (``/home/tristanlf/seabot2/seabot2-ros/src/seabot2_driver/gpsd_client/src/gpsd_node.cpp``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #include "gpsd_client/gpsd_node.h"
   
   using namespace placeholders;
   
   GpsdNode::GpsdNode()
   : Node("gpsd_node")
   {
   
     init_parameters();
     init_interfaces();
   
     gps_ = new gpsmm("localhost", DEFAULT_GPSD_PORT);
   
     if(gps_->stream(WATCH_ENABLE | WATCH_JSON | WATCH_PPS) == nullptr) {
       RCLCPP_WARN(this->get_logger(), "[gpsd_node] Failed to open GPSd");
       exit(EXIT_FAILURE);
     }
     RCLCPP_INFO(this->get_logger(), "[gpsd_node] GPSd opened");
     gps_->clear_fix();
   
     timer_ = this->create_wall_timer(
               loop_dt_, std::bind(&GpsdNode::timer_callback, this));
   
     RCLCPP_INFO(this->get_logger(), "[gpsd_node] Start Ok");
   }
   
   GpsdNode::~GpsdNode()
   {
     if(gps_ != nullptr) {
       gps_->stream(WATCH_DISABLE);
       gps_->~gpsmm();
       free(gps_);
     }
   }
   
   void GpsdNode::init_parameters()
   {
     this->declare_parameter<long>("loop_dt", loop_dt_.count());
     this->declare_parameter<string>("frame_id", frame_id_);
     this->declare_parameter<bool>("publish_when_no_fix", publish_when_no_fix_);
   
     frame_id_ = this->get_parameter_or("frame_id", frame_id_);
     publish_when_no_fix_ = this->get_parameter_or("publish_when_no_fix", publish_when_no_fix_);
     loop_dt_ = std::chrono::milliseconds(this->get_parameter_or("dt", loop_dt_.count()));
   }
   
   void GpsdNode::init_interfaces()
   {
     publisher_fix_ = this->create_publisher<seabot2_msgs::msg::GpsFix>("fix", 1);
     publisher_pps_ = this->create_publisher<seabot2_msgs::msg::GpsPps>("pps", 1);
   }
   
   void GpsdNode::timer_callback()
   {
     if (!gps_->waiting(2000000)) { // us ? => 2s
       return;
     }
   
     if(gps_data_t * p; (p = gps_->read()) == nullptr) {
       RCLCPP_WARN(this->get_logger(), "[gpsd_node] Error reading gpsd");
     } else {
       process_data(p);
     }
   }
   
   void GpsdNode::process_data(const gps_data_t * p)
   {
     if (!(p->set & PACKET_SET)) {  // ToDo : to be checked
       return;
     }
   
     seabot2_msgs::msg::GpsFix msg;
     msg.header.stamp = this->now();
     msg.header.frame_id = frame_id_;
     msg.status = p->fix.status;
     msg.mode = p->fix.mode;
     msg.time = p->fix.time.tv_sec + p->fix.time.tv_nsec * 1e-9;
   
     if(publish_when_no_fix_ || p->fix.mode >= MODE_2D) {
       msg.latitude = p->fix.latitude;
       msg.longitude = p->fix.longitude;
   
       msg.altitude = p->fix.altHAE;
   
       msg.track = p->fix.track;
       msg.speed = p->fix.speed;
   
       msg.pdop = p->dop.pdop;
       msg.hdop = p->dop.hdop;
       msg.vdop = p->dop.vdop;
       msg.tdop = p->dop.tdop;
       msg.gdop = p->dop.gdop;
   
       int nb_sat_visible = p->satellites_visible;
       msg.satellites_visible = nb_sat_visible;
   
       double max_snr = -1.0;
       double sum_snr = 0.0;
       int count_valid_snr = 0.0;
   
       std::vector<float> snr_values;
       for (int i = 0; i < nb_sat_visible; i++) {
         int prn = p->skyview[i].PRN;          // Satellite PRN (identifier)
         double snr = p->skyview[i].ss;           // Signal strength (SNR, dBHz)
         double el = p->skyview[i].elevation;
         double az = p->skyview[i].azimuth;
   
               // RCLCPP_INFO(this->get_logger(),
               //     "[gpsd_node] Sat %d: SNR=%d dBHz, Elev=%f°, Azim=%f°",
               //     prn, snr, el, az);
   
         if (snr > 0) {        // discard invalid SNRs
           sum_snr += snr;
           count_valid_snr++;
           if (snr > max_snr) {
             max_snr = snr;
           }
           snr_values.push_back(snr);
         }
       }
   
       msg.max_snr = max_snr;
       msg.mean_snr = (count_valid_snr > 0) ? (sum_snr / count_valid_snr) : -1.0;
   
           // --- Compute median SNR ---
       if (!snr_values.empty()) {
         std::sort(snr_values.begin(), snr_values.end());
         size_t mid = snr_values.size() / 2;
         if (snr_values.size() % 2 == 0) {
           msg.median_snr = (snr_values[mid - 1] + snr_values[mid]) / 2.0;
         } else {
           msg.median_snr = snr_values[mid];
         }
       } else {
         msg.median_snr = -1.0;       // or NaN if you prefer
       }
   
   
       if(!isnan(p->fix.eph)) {
         msg.err = p->fix.eph;
       }
       msg.err_horz = p->fix.eph;
       if(!isnan(p->fix.epv)) {
         msg.err_vert = p->fix.epv;
       }
       if(!isnan(p->fix.epd)) {
         msg.err_track = p->fix.epd;
       }
       if(!isnan(p->fix.eps)) {
         msg.err_speed = p->fix.eps;
       }
       if(!isnan(p->fix.ept)) {
         msg.err_time = p->fix.ept;
       }
   
       publisher_fix_->publish(msg);
       last_msg_no_fix_ = false;
     }
   
     if(!publish_when_no_fix_ && p->fix.mode < MODE_2D) {
       if(!last_msg_no_fix_) {
         publisher_fix_->publish(msg);
       }
       last_msg_no_fix_ = true;
     }
   
     if (p->set && PPS_SET) {
       seabot2_msgs::msg::GpsPps msg_pps;
       msg_pps.real_tv_sec = p->pps.real.tv_sec;
       msg_pps.real_tv_nsec = p->pps.real.tv_nsec;
       msg_pps.clock_tv_sec = p->pps.clock.tv_sec;
       msg_pps.clock_tv_nsec = p->pps.clock.tv_nsec;
       publisher_pps_->publish(msg_pps);
     }
   }
   
   int main(const int argc, char *argv[])
   {
     rclcpp::init(argc, argv);
     rclcpp::spin(std::make_shared<GpsdNode>());
     rclcpp::shutdown();
     return 0;
   }
