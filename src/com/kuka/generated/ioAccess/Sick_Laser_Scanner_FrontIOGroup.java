package com.kuka.generated.ioAccess;

import javax.inject.Inject;
import javax.inject.Singleton;

import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.ioModel.AbstractIOGroup;
import com.kuka.roboticsAPI.ioModel.IOTypes;

/**
 * Automatically generated class to abstract I/O access to I/O group <b>Sick_Laser_Scanner_Front</b>.<br>
 * <i>Please, do not modify!</i>
 * <p>
 * <b>I/O group description:</b><br>
 * sd
 */
@Singleton
public class Sick_Laser_Scanner_FrontIOGroup extends AbstractIOGroup
{
	/**
	 * Constructor to create an instance of class 'Sick_Laser_Scanner_Front'.<br>
	 * <i>This constructor is automatically generated. Please, do not modify!</i>
	 *
	 * @param controller
	 *            the controller, which has access to the I/O group 'Sick_Laser_Scanner_Front'
	 */
	@Inject
	public Sick_Laser_Scanner_FrontIOGroup(Controller controller)
	{
		super(controller, "Sick_Laser_Scanner_Front");

		addMockedInput("B_sick_trigger_schutzfeld", IOTypes.BOOLEAN, 1, false);
		addInput("A_sick_trigger_warnfeld", IOTypes.BOOLEAN, 1);
		addInput("A_sick_trigger_schutzfeld", IOTypes.BOOLEAN, 1);
		addMockedInput("B_sick_trigger_warnfeld", IOTypes.BOOLEAN, 1, false);
	}

	/**
	 * Gets the value of the <b>digital input '<i>B_sick_trigger_schutzfeld</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * True = 1 - Wenn Warnfeld betreten wurde, dann auf 1 
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital input 'B_sick_trigger_schutzfeld'
	* 
	 * @deprecated The output 'B_sick_trigger_schutzfeld' has not been assigned to a field bus address - thus this operation will be <b>simulated</b> only.
	 */
	@Deprecated
	public boolean getB_sick_trigger_schutzfeld()
	{
		return getBooleanIOValue("B_sick_trigger_schutzfeld", false);
	}

	/**
	 * Sets the value of the <b>mocked digital input '<i>B_sick_trigger_schutzfeld</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * True = 1 - Wenn Warnfeld betreten wurde, dann auf 1 
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @param value
	 *            the value, which has to be written to the mocked digital input 'B_sick_trigger_schutzfeld'
	* 
	 * @deprecated The output 'B_sick_trigger_schutzfeld' has not been assigned to a field bus address - thus this operation will be <b>simulated</b> only.
	 */
	@Deprecated
	public void setMockedB_sick_trigger_schutzfeldValue(java.lang.Boolean value)
	{
		setMockedInput("B_sick_trigger_schutzfeld", value);
	}

	/**
	 * Gets the value of the <b>digital input '<i>A_sick_trigger_warnfeld</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * True = 1 - Wenn Warnfeld betreten wurde, dann auf 1 
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital input 'A_sick_trigger_warnfeld'
	 */
	public boolean getA_sick_trigger_warnfeld()
	{
		return getBooleanIOValue("A_sick_trigger_warnfeld", false);
	}

	/**
	 * Gets the value of the <b>digital input '<i>A_sick_trigger_schutzfeld</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * True = 1 - Wenn Warnfeld betreten wurde, dann auf 1 
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital input 'A_sick_trigger_schutzfeld'
	 */
	public boolean getA_sick_trigger_schutzfeld()
	{
		return getBooleanIOValue("A_sick_trigger_schutzfeld", false);
	}

	/**
	 * Gets the value of the <b>digital input '<i>B_sick_trigger_warnfeld</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * True = 1 - Wenn Warnfeld betreten wurde, dann auf 1 
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital input 'B_sick_trigger_warnfeld'
	* 
	 * @deprecated The output 'B_sick_trigger_warnfeld' has not been assigned to a field bus address - thus this operation will be <b>simulated</b> only.
	 */
	@Deprecated
	public boolean getB_sick_trigger_warnfeld()
	{
		return getBooleanIOValue("B_sick_trigger_warnfeld", false);
	}

	/**
	 * Sets the value of the <b>mocked digital input '<i>B_sick_trigger_warnfeld</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * True = 1 - Wenn Warnfeld betreten wurde, dann auf 1 
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @param value
	 *            the value, which has to be written to the mocked digital input 'B_sick_trigger_warnfeld'
	* 
	 * @deprecated The output 'B_sick_trigger_warnfeld' has not been assigned to a field bus address - thus this operation will be <b>simulated</b> only.
	 */
	@Deprecated
	public void setMockedB_sick_trigger_warnfeldValue(java.lang.Boolean value)
	{
		setMockedInput("B_sick_trigger_warnfeld", value);
	}

}
