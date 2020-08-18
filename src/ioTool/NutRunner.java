package ioTool;

import com.kuka.common.ThreadUtil;
import com.kuka.generated.ioAccess.ExternalIOGroup;
import com.kuka.roboticsAPI.controllerModel.Controller;

public class NutRunner extends ExternalIOGroup {

	static final int NutInsert15N  = 1;
	static final int NutHomeLoose  = 7; 
	
	public NutRunner(Controller controller) {
		super(controller);
	}

	public void init() {
		
		setOUT11(false);  // SelectIO bit 0
		setOUT12(false);  // SelectIO bit 1
		setOUT13(false);  // SelectIO bit 2
		setOUT14(false);  // NutRunner CW
	}


	/********************* IO *******************/
	public boolean moveNutRunnerHomePos() {
		int nTryCount = 0;
		int nMaxTryCount = 500;
		boolean check = false;
		int repeat = 1;
		StopNutRunner();

		selectIO(NutInsert15N);
		ThreadUtil.milliSleep(300);

		for (int i = 0; i < 5; i++) {
			nTryCount =0;
			repeat = 1;
			RunNutRunnerCW();
			while (repeat > 0) {
				nTryCount++;
				if (CheckHome()) {
					repeat = 0;
					check = true;
				} else {
					if (nTryCount > nMaxTryCount)
						repeat = 0;
				}
				ThreadUtil.milliSleep(10);
			}

			StopNutRunner();
			ThreadUtil.milliSleep(500);
			if (check) {
				if (CheckHome())
					break;
				else{
					check =false;
				}
			} 
		}
		if(check) System.out.println("nut runner home check Ok");
		else System.out.println("nut runner home check Ng");

		return check;
	}
	
	public boolean CheckHome()
	{
		return getIN14();
	}

	public void RunNutRunnerCW() {
		setOUT14(true);
	}


	public void StopNutRunner() {
		setOUT14(false);
	}

	public boolean CheckTightenOK() {

		return getIN05();
	}

	public boolean CheckTightenNOK() {

		return getIN04();
	}
	
	public void selectIO(int ioNum)
	{
		switch(ioNum)
		{
		case 1:
			setOUT11(true);
			setOUT12(false);
			setOUT13(false);
			break;
		case 2:
			setOUT11(false);
			setOUT12(true);
			setOUT13(false);
			break;
		case 3:
			setOUT11(true);
			setOUT12(true);
			setOUT13(false);
			break;
		case 4:
			setOUT11(false);
			setOUT12(false);
			setOUT13(true);
			break;
		case 5:
			setOUT11(true);
			setOUT12(false);
			setOUT13(true);
			break;
		case 6:
			setOUT11(false);
			setOUT12(true);
			setOUT13(true);
			break;
		case 7:
			setOUT11(true);
			setOUT12(true);
			setOUT13(true);
			break;
		default :
			break;
		}
	}
	public void IOClear()
	{
		setOUT05(false);
		setOUT06(false);
		setOUT07(false);
		setOUT08(false);
		setOUT09(false);
		setOUT10(false);
		setOUT11(false);
		setOUT12(false);
		setOUT13(false);
		setOUT14(false);
		setOUT15(false);
		setOUT16(false);
	}


	/********************* IO END *******************/

}

