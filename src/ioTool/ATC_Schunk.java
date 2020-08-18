package ioTool;

import com.kuka.common.ThreadUtil;
import com.kuka.generated.ioAccess.ExternalIOGroup;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.controllerModel.Controller;

public class ATC_Schunk extends ExternalIOGroup{
		
	public ATC_Schunk(Controller controller) {
		super(controller);
		//init();
	}
	
	
	public void init()
	{
		// Set Gripper Open
		setOUT15(false);
		setOUT16(true);
	}
	
	public boolean CheckGripperClose()
	{
		return getIN15();
	}
	
	public boolean CheckGripperOpen()
	{
		return getIN16();
	}
	
	public void GripperClose()
	{
	
		setOUT15(false);
		setOUT16(true);
	}
	
	public void GripperOpen()
	{
		
		setOUT15(true);
		setOUT16(false);
	}
	
}
