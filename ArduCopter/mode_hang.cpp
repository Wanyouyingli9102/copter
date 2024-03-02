#include "Copter.h"
#include <GCS_MAVLink/GCS.h>
/*
 * Init and run calls for hang flight mode
 * writer lqd
 */

// stabilize_run - runs the main stabilize controller
// should be called at 100hz or more
void ModeHang::run()
{
    // apply simple mode transform to pilot inputs
    update_simple_mode();
    gcs().send_text(MAV_SEVERITY_CRITICAL,"mode hang fly");  //地面站消息发送

}
