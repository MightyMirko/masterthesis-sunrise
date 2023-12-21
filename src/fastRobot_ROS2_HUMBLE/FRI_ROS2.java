// Copyright 2022, ICube Laboratory, University of Strasbourg
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

package fastRobot_ROS2_HUMBLE;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import javax.inject.Inject;
//import javax.inject.Named;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

//import com.kuka.roboticsAPI.conditionModel.BooleanIOCondition;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
//import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.motionModel.PositionHold;
import com.kuka.roboticsAPI.motionModel.controlModeModel.JointImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.PositionControlMode;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;
import com.kuka.connectivity.fastRobotInterface.ClientCommandMode;
import com.kuka.connectivity.fastRobotInterface.FRIChannelInformation;
import com.kuka.connectivity.fastRobotInterface.FRIConfiguration;
import com.kuka.connectivity.fastRobotInterface.FRIJointOverlay;
import com.kuka.connectivity.fastRobotInterface.FRISession;
import com.kuka.connectivity.fastRobotInterface.IFRISessionListener;

import fastRobot_ROS2_HUMBLE.AngleConverter;



/**
 * Implementation of a robot application.
 * <p>
 * The application provides a {@link RoboticsAPITask#initialize()} and a
 * {@link RoboticsAPITask#run()} method, which will be called successively in
 * the application lifecycle. The application will terminate automatically after
 * the {@link RoboticsAPITask#run()} method has finished or after stopping the
 * task. The {@link RoboticsAPITask#dispose()} method will be called, even if an
 * exception is thrown during initialization or run.
 * <p>
 * <b>It is imperative to call <code>super.dispose()</code> when overriding the
 * {@link RoboticsAPITask#dispose()} method.</b>
 *
 * @see UseRoboticsAPIContext
 * @see #initialize()
 * @see #run()
 * @see #dispose()
 */
public class FRI_ROS2 extends RoboticsAPIApplication {
	private Controller _lbrController;
    private LBR _lbr;
    private String _clientName;
    @Inject
    //private MediaFlangeIOGroup _medflange;

    PositionControlMode ctrMode = new PositionControlMode();
    PositionHold posHold = new PositionHold(ctrMode, -1, TimeUnit.MINUTES);
    FRIJointOverlay jointOverlay;
	static double[] degrees = {
		0.0, 0.0, 0.0,
		0.0, 0.0, 0.0,
		0.0
		};
    static double[] radians = AngleConverter.degreesToRadians(degrees);

	private static final JointPosition INITIAL_POSITION = new JointPosition(
            radians[0], radians[1], radians[2],
            radians[3], radians[4], radians[5],
            radians[6]
        );
    private static final String CLIENT_IP = "172.31.0.21";
	private static final int TS = 5; //in ms

    IFRISessionListener listener = new IFRISessionListener(){
    	@Override
    	public void onFRIConnectionQualityChanged(
    	FRIChannelInformation friChannelInformation){
    	getLogger().info("QualityChangedEvent - quality:" +
    	friChannelInformation.getQuality()+"\n Jitter info:" + friChannelInformation.getJitter() +"\n Latency info:" + friChannelInformation.getLatency());
    	}
    	@Override
    	public void onFRISessionStateChanged(
    	FRIChannelInformation friChannelInformation){
    	getLogger().info("SessionStateChangedEvent - session state:" +
    	friChannelInformation.getFRISessionState() +"\n Jitter info:" + friChannelInformation.getJitter() +"\n Latency info:" + friChannelInformation.getLatency());
    	}
    	};

	@Override
	public void initialize() {
		_lbrController = (Controller) getContext().getControllers().toArray()[0];
        _lbr = (LBR) _lbrController.getDevices().toArray()[0];
        // **********************************************************************
        // *** change next line to the FRIClient's IP address                 ***
        // **********************************************************************
        _clientName = CLIENT_IP;
        //_lbr.attachTo(_lbr.getFlange()); eaaeiu
	}


	@Override
	public void run() {
        // Select the type of control
		String ques = "Select FRI control mode  :\n";
    	double res = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION,ques , "POSITION","TORQUE","MONITORING","Cancel");

		//_medflange.setLEDRed(true);
    	double speed_init = 0.7;
		_lbr.move(ptp(INITIAL_POSITION).setJointVelocityRel(speed_init));

		if(res == 0){
			PositionControlMode ctrMode = new PositionControlMode();
			posHold = new PositionHold(ctrMode, -1, TimeUnit.MINUTES);
		}
		else if (res == 1 || res == 2){
			JointImpedanceControlMode ctrMode = new JointImpedanceControlMode(0.0,0.0,0.0,0.0,0.0,0.0,0.0);
			ctrMode.setStiffnessForAllJoints(0.0);
			posHold = new PositionHold(ctrMode, -1, TimeUnit.MINUTES);
		}
		else return;

        // configure and start FRI session
        FRIConfiguration friConfiguration = FRIConfiguration.createRemoteConfiguration(_lbr, _clientName);
        // for torque mode, there has to be a command value at least all 5ms
        friConfiguration.setSendPeriodMilliSec((int) TS);
        friConfiguration.setReceiveMultiplier(1);

        getLogger().info("Creating FRI connection to " + friConfiguration.getHostName());
        getLogger().info("SendPeriod: " + friConfiguration.getSendPeriodMilliSec() + "ms |"
                + " ReceiveMultiplier: " + friConfiguration.getReceiveMultiplier());

        FRISession friSession = new FRISession(friConfiguration);
        friSession.addFRISessionListener(listener);

        // wait until FRI session is ready to switch to command mode
        try
        {
            friSession.await(20, TimeUnit.SECONDS);
        }
        catch (final TimeoutException e)
        {
            getLogger().error(e.getLocalizedMessage());
            friSession.close();
            return;
        }

        getLogger().info("FRI connection established.");

        getLogger().info("Jitter info: " + friSession.getFRIChannelInformation().getJitter());

        if(res == 0){
	    	jointOverlay = new FRIJointOverlay(friSession, ClientCommandMode.POSITION);
		}
		else if (res == 1){
			jointOverlay = new FRIJointOverlay(friSession, ClientCommandMode.TORQUE);
		}
		else if(res== 2){
			jointOverlay = new FRIJointOverlay(friSession, ClientCommandMode.NO_COMMAND_MODE);
		}
		else return;


        //_medflange.setLEDRed(false);
        //_medflange.setLEDGreen(true);
        //BooleanIOCondition _buttonPressed = new BooleanIOCondition(_medflange.getInput("UserButton"), true);
        
        if(res == 0 || res == 1) _lbr.move(posHold.addMotionOverlay(jointOverlay));
        else  _lbr.move(posHold);

		//_medflange.setLEDGreen(false);
		//_medflange.setLEDRed(true);
        // done
        friSession.close();
        getLogger().info("FRI connection closed.");
        getLogger().info("Application stopped.");
	}

	public static void main(final String[] args)
    {
        final FRI_ROS2 app = new FRI_ROS2();
        app.runApplication();
    }
}
