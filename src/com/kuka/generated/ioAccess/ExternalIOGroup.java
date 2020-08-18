package com.kuka.generated.ioAccess;

import javax.inject.Inject;
import javax.inject.Singleton;

import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.ioModel.AbstractIOGroup;
import com.kuka.roboticsAPI.ioModel.IOTypes;
import com.kuka.roboticsAPI.ioModel.OutputReservedException;

/**
 * Automatically generated class to abstract I/O access to I/O group <b>External</b>.<br>
 * <i>Please, do not modify!</i>
 * <p>
 * <b>I/O group description:</b><br>
 * ./.
 */
@Singleton
public class ExternalIOGroup extends AbstractIOGroup
{
	/**
	 * Constructor to create an instance of class 'External'.<br>
	 * <i>This constructor is automatically generated. Please, do not modify!</i>
	 *
	 * @param controller
	 *            the controller, which has access to the I/O group 'External'
	 */
	@Inject
	public ExternalIOGroup(Controller controller)
	{
		super(controller, "External");

		addInput("IN01", IOTypes.BOOLEAN, 1);
		addInput("IN02", IOTypes.BOOLEAN, 1);
		addInput("IN03", IOTypes.BOOLEAN, 1);
		addInput("IN04", IOTypes.BOOLEAN, 1);
		addMockedDigitalOutput("OUT01", IOTypes.BOOLEAN, 1);
		addMockedDigitalOutput("OUT02", IOTypes.BOOLEAN, 1);
		addMockedDigitalOutput("OUT03", IOTypes.BOOLEAN, 1);
		addMockedDigitalOutput("OUT04", IOTypes.BOOLEAN, 1);
		addDigitalOutput("OUT05", IOTypes.BOOLEAN, 1);
		addDigitalOutput("OUT06", IOTypes.BOOLEAN, 1);
		addDigitalOutput("OUT15", IOTypes.BOOLEAN, 1);
		addDigitalOutput("OUT16", IOTypes.BOOLEAN, 1);
		addInput("IN05", IOTypes.BOOLEAN, 1);
		addInput("IN06", IOTypes.BOOLEAN, 1);
		addInput("IN07", IOTypes.BOOLEAN, 1);
		addInput("IN08", IOTypes.BOOLEAN, 1);
		addInput("IN09", IOTypes.BOOLEAN, 1);
		addInput("IN10", IOTypes.BOOLEAN, 1);
		addInput("IN11", IOTypes.BOOLEAN, 1);
		addInput("IN12", IOTypes.BOOLEAN, 1);
		addInput("IN13", IOTypes.BOOLEAN, 1);
		addInput("IN14", IOTypes.BOOLEAN, 1);
		addInput("IN15", IOTypes.BOOLEAN, 1);
		addInput("IN16", IOTypes.BOOLEAN, 1);
		addDigitalOutput("OUT07", IOTypes.BOOLEAN, 1);
		addDigitalOutput("OUT08", IOTypes.BOOLEAN, 1);
		addDigitalOutput("OUT09", IOTypes.BOOLEAN, 1);
		addDigitalOutput("OUT10", IOTypes.BOOLEAN, 1);
		addDigitalOutput("OUT11", IOTypes.BOOLEAN, 1);
		addDigitalOutput("OUT12", IOTypes.BOOLEAN, 1);
		addDigitalOutput("OUT13", IOTypes.BOOLEAN, 1);
		addDigitalOutput("OUT14", IOTypes.BOOLEAN, 1);
	}

	/**
	 * Gets the value of the <b>digital input '<i>IN01</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital input 'IN01'
	 */
	public boolean getIN01()
	{
		return getBooleanIOValue("IN01", false);
	}

	/**
	 * Gets the value of the <b>digital input '<i>IN02</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital input 'IN02'
	 */
	public boolean getIN02()
	{
		return getBooleanIOValue("IN02", false);
	}

	/**
	 * Gets the value of the <b>digital input '<i>IN03</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital input 'IN03'
	 */
	public boolean getIN03()
	{
		return getBooleanIOValue("IN03", false);
	}

	/**
	 * Gets the value of the <b>digital input '<i>IN04</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital input 'IN04'
	 */
	public boolean getIN04()
	{
		return getBooleanIOValue("IN04", false);
	}

	/**
	 * Gets the value of the <b>digital output '<i>OUT01</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital output 'OUT01'
	* 
	 * @deprecated The output 'OUT01' is currently used as station state output in the Sunrise project properties.
	 */
	@Deprecated
	public boolean getOUT01()
	{
		return getBooleanIOValue("OUT01", true);
	}

	/**
	 * Always throws an {@code OutputReservedException}, because the <b>digital output '<i>OUT01</i>'</b> is currently used as station state output in the Sunrise project properties.
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @param value
	 *            the value, which has to be written to the digital output 'OUT01'
	 * @throws OutputReservedException
	 *            Always thrown, because this output is currently used as station state output in the Sunrise project properties.
	* 
	 * @deprecated The output 'OUT01' is currently used as station state output in the Sunrise project properties.
	 */
	@Deprecated
	public void setOUT01(java.lang.Boolean value) throws OutputReservedException
	{
		throw new OutputReservedException("The output 'OUT01' must not be set because it is currently used as station state output in the Sunrise project properties.");
	}

	/**
	 * Gets the value of the <b>digital output '<i>OUT02</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital output 'OUT02'
	* 
	 * @deprecated The output 'OUT02' is currently used as station state output in the Sunrise project properties.
	 */
	@Deprecated
	public boolean getOUT02()
	{
		return getBooleanIOValue("OUT02", true);
	}

	/**
	 * Always throws an {@code OutputReservedException}, because the <b>digital output '<i>OUT02</i>'</b> is currently used as station state output in the Sunrise project properties.
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @param value
	 *            the value, which has to be written to the digital output 'OUT02'
	 * @throws OutputReservedException
	 *            Always thrown, because this output is currently used as station state output in the Sunrise project properties.
	* 
	 * @deprecated The output 'OUT02' is currently used as station state output in the Sunrise project properties.
	 */
	@Deprecated
	public void setOUT02(java.lang.Boolean value) throws OutputReservedException
	{
		throw new OutputReservedException("The output 'OUT02' must not be set because it is currently used as station state output in the Sunrise project properties.");
	}

	/**
	 * Gets the value of the <b>digital output '<i>OUT03</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital output 'OUT03'
	* 
	 * @deprecated The output 'OUT03' is currently used as station state output in the Sunrise project properties.
	 */
	@Deprecated
	public boolean getOUT03()
	{
		return getBooleanIOValue("OUT03", true);
	}

	/**
	 * Always throws an {@code OutputReservedException}, because the <b>digital output '<i>OUT03</i>'</b> is currently used as station state output in the Sunrise project properties.
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @param value
	 *            the value, which has to be written to the digital output 'OUT03'
	 * @throws OutputReservedException
	 *            Always thrown, because this output is currently used as station state output in the Sunrise project properties.
	* 
	 * @deprecated The output 'OUT03' is currently used as station state output in the Sunrise project properties.
	 */
	@Deprecated
	public void setOUT03(java.lang.Boolean value) throws OutputReservedException
	{
		throw new OutputReservedException("The output 'OUT03' must not be set because it is currently used as station state output in the Sunrise project properties.");
	}

	/**
	 * Gets the value of the <b>digital output '<i>OUT04</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital output 'OUT04'
	* 
	 * @deprecated The output 'OUT04' is currently used as station state output in the Sunrise project properties.
	 */
	@Deprecated
	public boolean getOUT04()
	{
		return getBooleanIOValue("OUT04", true);
	}

	/**
	 * Always throws an {@code OutputReservedException}, because the <b>digital output '<i>OUT04</i>'</b> is currently used as station state output in the Sunrise project properties.
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @param value
	 *            the value, which has to be written to the digital output 'OUT04'
	 * @throws OutputReservedException
	 *            Always thrown, because this output is currently used as station state output in the Sunrise project properties.
	* 
	 * @deprecated The output 'OUT04' is currently used as station state output in the Sunrise project properties.
	 */
	@Deprecated
	public void setOUT04(java.lang.Boolean value) throws OutputReservedException
	{
		throw new OutputReservedException("The output 'OUT04' must not be set because it is currently used as station state output in the Sunrise project properties.");
	}

	/**
	 * Gets the value of the <b>digital output '<i>OUT05</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital output 'OUT05'
	 */
	public boolean getOUT05()
	{
		return getBooleanIOValue("OUT05", true);
	}

	/**
	 * Sets the value of the <b>digital output '<i>OUT05</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @param value
	 *            the value, which has to be written to the digital output 'OUT05'
	 */
	public void setOUT05(java.lang.Boolean value)
	{
		setDigitalOutput("OUT05", value);
	}

	/**
	 * Gets the value of the <b>digital output '<i>OUT06</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital output 'OUT06'
	 */
	public boolean getOUT06()
	{
		return getBooleanIOValue("OUT06", true);
	}

	/**
	 * Sets the value of the <b>digital output '<i>OUT06</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @param value
	 *            the value, which has to be written to the digital output 'OUT06'
	 */
	public void setOUT06(java.lang.Boolean value)
	{
		setDigitalOutput("OUT06", value);
	}

	/**
	 * Gets the value of the <b>digital output '<i>OUT15</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital output 'OUT15'
	 */
	public boolean getOUT15()
	{
		return getBooleanIOValue("OUT15", true);
	}

	/**
	 * Sets the value of the <b>digital output '<i>OUT15</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @param value
	 *            the value, which has to be written to the digital output 'OUT15'
	 */
	public void setOUT15(java.lang.Boolean value)
	{
		setDigitalOutput("OUT15", value);
	}

	/**
	 * Gets the value of the <b>digital output '<i>OUT16</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital output 'OUT16'
	 */
	public boolean getOUT16()
	{
		return getBooleanIOValue("OUT16", true);
	}

	/**
	 * Sets the value of the <b>digital output '<i>OUT16</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @param value
	 *            the value, which has to be written to the digital output 'OUT16'
	 */
	public void setOUT16(java.lang.Boolean value)
	{
		setDigitalOutput("OUT16", value);
	}

	/**
	 * Gets the value of the <b>digital input '<i>IN05</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital input 'IN05'
	 */
	public boolean getIN05()
	{
		return getBooleanIOValue("IN05", false);
	}

	/**
	 * Gets the value of the <b>digital input '<i>IN06</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital input 'IN06'
	 */
	public boolean getIN06()
	{
		return getBooleanIOValue("IN06", false);
	}

	/**
	 * Gets the value of the <b>digital input '<i>IN07</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital input 'IN07'
	 */
	public boolean getIN07()
	{
		return getBooleanIOValue("IN07", false);
	}

	/**
	 * Gets the value of the <b>digital input '<i>IN08</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital input 'IN08'
	 */
	public boolean getIN08()
	{
		return getBooleanIOValue("IN08", false);
	}

	/**
	 * Gets the value of the <b>digital input '<i>IN09</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital input 'IN09'
	 */
	public boolean getIN09()
	{
		return getBooleanIOValue("IN09", false);
	}

	/**
	 * Gets the value of the <b>digital input '<i>IN10</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital input 'IN10'
	 */
	public boolean getIN10()
	{
		return getBooleanIOValue("IN10", false);
	}

	/**
	 * Gets the value of the <b>digital input '<i>IN11</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital input 'IN11'
	 */
	public boolean getIN11()
	{
		return getBooleanIOValue("IN11", false);
	}

	/**
	 * Gets the value of the <b>digital input '<i>IN12</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital input 'IN12'
	 */
	public boolean getIN12()
	{
		return getBooleanIOValue("IN12", false);
	}

	/**
	 * Gets the value of the <b>digital input '<i>IN13</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital input 'IN13'
	 */
	public boolean getIN13()
	{
		return getBooleanIOValue("IN13", false);
	}

	/**
	 * Gets the value of the <b>digital input '<i>IN14</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital input 'IN14'
	 */
	public boolean getIN14()
	{
		return getBooleanIOValue("IN14", false);
	}

	/**
	 * Gets the value of the <b>digital input '<i>IN15</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital input 'IN15'
	 */
	public boolean getIN15()
	{
		return getBooleanIOValue("IN15", false);
	}

	/**
	 * Gets the value of the <b>digital input '<i>IN16</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital input 'IN16'
	 */
	public boolean getIN16()
	{
		return getBooleanIOValue("IN16", false);
	}

	/**
	 * Gets the value of the <b>digital output '<i>OUT07</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital output 'OUT07'
	 */
	public boolean getOUT07()
	{
		return getBooleanIOValue("OUT07", true);
	}

	/**
	 * Sets the value of the <b>digital output '<i>OUT07</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @param value
	 *            the value, which has to be written to the digital output 'OUT07'
	 */
	public void setOUT07(java.lang.Boolean value)
	{
		setDigitalOutput("OUT07", value);
	}

	/**
	 * Gets the value of the <b>digital output '<i>OUT08</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital output 'OUT08'
	 */
	public boolean getOUT08()
	{
		return getBooleanIOValue("OUT08", true);
	}

	/**
	 * Sets the value of the <b>digital output '<i>OUT08</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @param value
	 *            the value, which has to be written to the digital output 'OUT08'
	 */
	public void setOUT08(java.lang.Boolean value)
	{
		setDigitalOutput("OUT08", value);
	}

	/**
	 * Gets the value of the <b>digital output '<i>OUT09</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital output 'OUT09'
	 */
	public boolean getOUT09()
	{
		return getBooleanIOValue("OUT09", true);
	}

	/**
	 * Sets the value of the <b>digital output '<i>OUT09</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @param value
	 *            the value, which has to be written to the digital output 'OUT09'
	 */
	public void setOUT09(java.lang.Boolean value)
	{
		setDigitalOutput("OUT09", value);
	}

	/**
	 * Gets the value of the <b>digital output '<i>OUT10</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital output 'OUT10'
	 */
	public boolean getOUT10()
	{
		return getBooleanIOValue("OUT10", true);
	}

	/**
	 * Sets the value of the <b>digital output '<i>OUT10</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @param value
	 *            the value, which has to be written to the digital output 'OUT10'
	 */
	public void setOUT10(java.lang.Boolean value)
	{
		setDigitalOutput("OUT10", value);
	}

	/**
	 * Gets the value of the <b>digital output '<i>OUT11</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital output 'OUT11'
	 */
	public boolean getOUT11()
	{
		return getBooleanIOValue("OUT11", true);
	}

	/**
	 * Sets the value of the <b>digital output '<i>OUT11</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @param value
	 *            the value, which has to be written to the digital output 'OUT11'
	 */
	public void setOUT11(java.lang.Boolean value)
	{
		setDigitalOutput("OUT11", value);
	}

	/**
	 * Gets the value of the <b>digital output '<i>OUT12</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital output 'OUT12'
	 */
	public boolean getOUT12()
	{
		return getBooleanIOValue("OUT12", true);
	}

	/**
	 * Sets the value of the <b>digital output '<i>OUT12</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @param value
	 *            the value, which has to be written to the digital output 'OUT12'
	 */
	public void setOUT12(java.lang.Boolean value)
	{
		setDigitalOutput("OUT12", value);
	}

	/**
	 * Gets the value of the <b>digital output '<i>OUT13</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital output 'OUT13'
	 */
	public boolean getOUT13()
	{
		return getBooleanIOValue("OUT13", true);
	}

	/**
	 * Sets the value of the <b>digital output '<i>OUT13</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @param value
	 *            the value, which has to be written to the digital output 'OUT13'
	 */
	public void setOUT13(java.lang.Boolean value)
	{
		setDigitalOutput("OUT13", value);
	}

	/**
	 * Gets the value of the <b>digital output '<i>OUT14</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital output 'OUT14'
	 */
	public boolean getOUT14()
	{
		return getBooleanIOValue("OUT14", true);
	}

	/**
	 * Sets the value of the <b>digital output '<i>OUT14</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @param value
	 *            the value, which has to be written to the digital output 'OUT14'
	 */
	public void setOUT14(java.lang.Boolean value)
	{
		setDigitalOutput("OUT14", value);
	}

}
