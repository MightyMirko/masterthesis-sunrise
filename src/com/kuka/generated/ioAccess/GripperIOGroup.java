package com.kuka.generated.ioAccess;

import javax.inject.Inject;
import javax.inject.Singleton;

import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.ioModel.AbstractIOGroup;
import com.kuka.roboticsAPI.ioModel.IOTypes;

/**
 * Automatically generated class to abstract I/O access to I/O group <b>Gripper</b>.<br>
 * <i>Please, do not modify!</i>
 * <p>
 * <b>I/O group description:</b><br>
 * coact egp

 */
@Singleton
public class GripperIOGroup extends AbstractIOGroup
{
	/**
	 * Constructor to create an instance of class 'Gripper'.<br>
	 * <i>This constructor is automatically generated. Please, do not modify!</i>
	 *
	 * @param controller
	 *            the controller, which has access to the I/O group 'Gripper'
	 */
	@Inject
	public GripperIOGroup(Controller controller)
	{
		super(controller, "Gripper");

		addInput("Isopen", IOTypes.BOOLEAN, 1);
		addInput("Isclosed", IOTypes.BOOLEAN, 1);
		addDigitalOutput("SetClose", IOTypes.BOOLEAN, 1);
		addDigitalOutput("SetOpen", IOTypes.BOOLEAN, 1);
	}

	/**
	 * Gets the value of the <b>digital input '<i>Isopen</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * is gripper open
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital input 'Isopen'
	 */
	public boolean getIsopen()
	{
		return getBooleanIOValue("Isopen", false);
	}

	/**
	 * Gets the value of the <b>digital input '<i>Isclosed</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * is gripper closed
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital input 'Isclosed'
	 */
	public boolean getIsclosed()
	{
		return getBooleanIOValue("Isclosed", false);
	}

	/**
	 * Gets the value of the <b>digital output '<i>SetClose</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * Setting the Gripper to close the claws
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital output 'SetClose'
	 */
	public boolean getSetClose()
	{
		return getBooleanIOValue("SetClose", true);
	}

	/**
	 * Sets the value of the <b>digital output '<i>SetClose</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * Setting the Gripper to close the claws
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @param value
	 *            the value, which has to be written to the digital output 'SetClose'
	 */
	public void setSetClose(java.lang.Boolean value)
	{
		setDigitalOutput("SetClose", value);
	}

	/**
	 * Gets the value of the <b>digital output '<i>SetOpen</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * Setting the Gripper to open the claws
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital output 'SetOpen'
	 */
	public boolean getSetOpen()
	{
		return getBooleanIOValue("SetOpen", true);
	}

	/**
	 * Sets the value of the <b>digital output '<i>SetOpen</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * Setting the Gripper to open the claws
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @param value
	 *            the value, which has to be written to the digital output 'SetOpen'
	 */
	public void setSetOpen(java.lang.Boolean value)
	{
		setDigitalOutput("SetOpen", value);
	}

}
