package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

/**
 * Created by FIRSTMentor on 3/19/2017.
 */
public class VuforiaOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        VuforiaLocalizer.Parameters Param = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        Param.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        Param.vuforiaLicenseKey = "AQK0ORH/////AAAAGd4TUrj38EsdgUdvt5TMQJ4SY8JMs1KOiZvMJtPR6V7McoCsxbp0LIkjYNbGHNWXjydIfdG5rVhCjEYlQJS+e2IIoLTFJfLCHuKY+4fk4cXQPE3Iv8AKL77aKjdF0brznejYLpAbhDl5BLd/loeUXI1KckkNGyPm4/Q46W7T+u9sgrP0IGkRkIvWvlrvWBnEKrTos9x+bDRg9eqT8w9ClnjOtGt/E1xeAey4onNpO75ewJ3aTZoiXds8UVfDqSdY6IR43+zory6R7tE8kPp0LPaAn87aBVn+tZPD3FpCk+L95i6oVyGo+ugYqD49tIbOy8hqiu8rj4QjsY3ok2Lw2EJCMGLl7fKlzPKVZrBb072u";
    }
}
