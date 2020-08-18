package additionFunction;

import java.io.BufferedInputStream;
import java.io.BufferedOutputStream;
import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.io.OutputStreamWriter;
import java.io.PrintWriter;
import java.net.InetAddress;
import java.net.Socket;
import java.net.SocketException;
import java.util.Timer;
import java.util.TimerTask;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;

import javax.inject.Inject;

import com.kuka.common.ThreadUtil;
import com.kuka.generated.ioAccess.ExternalIOGroup;
import com.kuka.roboticsAPI.controllerModel.Controller;

public class NutCommThread implements Runnable{
	private boolean THREADFLAG;
	private CallbackReceive icbReceive;
	private ExternalIOGroup exio;
	private Thread rT;
    private Socket client_socket;
    private Socket socket;
    private static BufferedReader in;
    private static PrintWriter out;
	boolean end_flag = false;
	private String ip;

	private int port;
    String threadedMessage;
	public byte[] carInfos;
	
	int PSET_STATE = 0;
	
	Timer mTimer;
	TimerTask mTimerTask;
	
	public NutCommThread(String ip, int port) {
		// TODO Auto-generated constructor stub
		this.ip = ip;
		this.port = port;
		mTimer = new Timer();
		mTimerTask = new TimerTask(){
			@Override
			public void run() {
				// TODO Auto-generated method stub
				//System.out.println("send KeepAlive & Touque Result");
				if(checkComm()){
					sendByte(MakeStr(4, "0").getBytes());
				}
			}
		};

	}
	
	public void UsingCallBack(CallbackReceive _CbReceive){
		icbReceive = _CbReceive;
	}
	public void UsingExio(ExternalIOGroup exio){

		this.exio = exio;
	}
	
	public void startComm() {
		System.out.println("Starting NutTcp communication");
		try{
	    	client_socket = null;
			int maxretry = 100;
			
			while ( maxretry-- > 0 ) {
				try {
					client_socket = new Socket(InetAddress.getByName(ip), port);
					THREADFLAG = true;
					break;	// try succeed
				} catch (Exception e) {
				
				System.out.println("port " + port + " is in use");
				} finally {	}
			}	// end of while
				
			System.out.println("Server is ready on port [" + client_socket.getLocalPort() + "]" );
			System.out.println("TCP\tC : Wating for connection...");
			
			socket = client_socket;
			System.out.println("got a connection from [" + socket.getInetAddress() + "]\n");
			out = new PrintWriter(new BufferedWriter(new OutputStreamWriter(socket.getOutputStream())), true);
			in = new BufferedReader(new InputStreamReader(socket.getInputStream()));
			
			sendByte(MakeStr(0, "0").getBytes() );
			mTimer.cancel();
			mTimer = new Timer();
			mTimer.schedule(mTimerTask, 0, 500);
		} catch (IOException e) {
			System.out.println("NutTcp Server : Error" + e);
	    } catch(Exception ex) {
	    	
	    }
			 
	}
	
	public void killComm(){
		try {
        	System.out.println("Ending NutTCP communication");
        	exio = null;
        	icbReceive = null;
			THREADFLAG = false;
			mTimer.cancel();
			mTimerTask.cancel();
			//mTimer = null;
			//mTimerTask = null;
			end_flag = true;
        	socket.close();
			client_socket.close();
			try {
				rT.interrupt();
			} catch (Exception e) {
			}				
		} catch (IOException e) {
			//e.printStackTrace();
		} catch (Exception ex) {
			
		}
		
	}
	
	public boolean checkComm() {
		return socket.isConnected();
	}
	
	// socket thread
    public class ReceiveRunnable implements Runnable {
    	public String message;

        public String getMessage() {
			return message;
		}

		public void run() {
        	while ( !Thread.currentThread().isInterrupted() && !end_flag ) {
    			try {
					if ( (message = in.readLine()) != null ) { // receiving message via inputstream
						threadedMessage = message;
					}
				} catch (IOException e) {
//					e.printStackTrace();
				} catch (Exception ex) {
					
				}
        	}
        }	// end of run()

    }
	
    
    
    /**
	 * @param message : message to send to client
	 */
	public void send(String message) {
        try {
			System.out.println("TCP Server [SEND] : \"" + message + "\"");
			out.println(message); // sending message via outputstream
		} catch (Exception e) {
			e.printStackTrace();
		}
	}
	
	 public String getThreadedMessage() {
			return threadedMessage;
		}
	
	/**
	 * use this method once, and then use <b>getThreadedMessage()</b> method to get messages continuously.
	 */
	public void receiveWithThread() {
		threadedMessage = "";
		
		ReceiveRunnable rR = new ReceiveRunnable();
		rT = new Thread(rR);
		rT.start();
		
	}
	
	public void sendByte(byte[] data){
		DataOutputStream dos;
		
		try{
			dos = new DataOutputStream(socket.getOutputStream());
			dos.write(data);
		}catch(SocketException se) {
			System.out.println("Disconnceted");
			startComm();
		}catch(IOException e){
			e.printStackTrace();
		}catch(Exception ex) {
			
		}
	}
	
	public byte[] receiveByteWait(int bufferSize) {
    	try {
    		// clear buffered stacks
    		while( socket.getInputStream().available() != 0 ) {
				socket.getInputStream().read();
			}
    		
    		//System.out.println("Waiting for a message from Server...");
    		
			DataInputStream dis = new DataInputStream(socket.getInputStream());
			
			byte[] data = new byte[bufferSize];
			dis.read(data, 0, bufferSize);		// receiving message via inputstream
			
			return data;
		} catch (IOException e) {
			//e.printStackTrace();
		} catch (Exception ex) {
			
		}
    	
		return null;
	}
	
	public String receiveWait() {
		try {
			// clear buffered stacks
			while( socket.getInputStream().available() != 0 ) {
				socket.getInputStream().read();
			}

			System.out.println("Waiting for a message from Client...");

			String message;
			if ( (message = in.readLine()) != null ) { // receiving message via inputstream
				// message = in.readLine();
				System.out.println("TCP Server [RECEIVE] : \"" + message + "\"");
				return message;
			}
			//			}
		} catch (IOException e) {
			//e.printStackTrace();
		} catch (Exception ex) {
			
		}
    	
		return null;
	}

	public String receiveLines() {
		String content = "";
		
    	try {
    		// clear buffered stacks
    		while( socket.getInputStream().available() != 0 ) {
				socket.getInputStream().read();
			}
    		
    		System.out.println("Waiting for a message from Client...");
    		
    		do {
    			content += in.readLine();	// receiving message via inputstream
			} while (content.contains("END"));
    		
		} catch (IOException e) {
			//e.printStackTrace();
		}
    	
		return content;
	}
	
	public void sendFile() throws Exception {
        // Specify the file
		File file = new File("C:\\data.xml");
		FileInputStream fis = new FileInputStream(file);
		BufferedInputStream bis = new BufferedInputStream(fis);

        //Get socket's output stream
        OutputStream os = socket.getOutputStream();
        
        //Read File Contents into contents array 
        byte[] contents;
        long fileLength = file.length(); 
        long current = 0;
        
        long start = System.nanoTime();
        while(current!=fileLength){ 
            int size = 10000;
            if(fileLength - current >= size)
                current += size;    
            else{ 
                size = (int)(fileLength - current); 
                current = fileLength;
            } 
            contents = new byte[size]; 
            bis.read(contents, 0, size); 
            os.write(contents);
            System.out.print("Sending file ... "+(current*100)/fileLength+"% complete!");
        }   
        
        os.flush(); 
	}
	
	public void receiveFile() throws Exception {
        byte[] contents = new byte[1000000];	// 1MB
        
        //Initialize the FileOutputStream to the output file's full path.
        FileOutputStream fos = new FileOutputStream("E:\\data.xml");
        BufferedOutputStream bos = new BufferedOutputStream(fos);
        InputStream is = socket.getInputStream();
        
        //No of bytes read in one read() call
        int bytesRead = 0;

        while( ( bytesRead=is.read(contents) )!=-1 ) {
        	bos.write(contents, 0, bytesRead);         	
        }
        
        bos.flush();
        System.out.println("File saved successfully!");
	}
    
	
	public void onSendNutProgramSel(int ProgramNo) {
		System.out.println("Send: "+ MakeStr(5, String.format("%d", ProgramNo)));
		sendByte(MakeStr(5, String.format("%d", ProgramNo)).getBytes());
		//sendByte(MakeStr(11, "0").getBytes());
	}
	public void onSendNutStart() {
		System.out.println("Send: "+ MakeStr(6, "0"));
		sendByte(MakeStr(6, "0").getBytes());
		//ThreadUtil.milliSleep(500);
	}
	public void onSendNutStop() {
		sendByte(MakeStr(7, "0").getBytes());
		sendByte(MakeStr(8, "0").getBytes());
	}
	
	public String MakeStr(int Num_Cmd, String ProgramNo)
    {

    	String sMsg;
    	String strLength = "0020";
        String strMid = "0001";
        String strRevision = "000";
        String strAckFlag = "0";
        String strStatinID = "00";
        String strPindleID = "00";
        String strSpare = "0000";
        String strPset = "";
        String strMsgEnd = "\0";

        switch (Num_Cmd)
        {
            case 0: // Start 0020 0001 000 0 00 00 0000 \0
                strLength = "0020";
                strMid = "0001";
                strRevision = "003";
                break;
            case 1: // Stop 0020 0003 000 0 00 00 0000 \0
                strLength = "0020";
                strMid = "0003";
                strRevision = "001";
                break;
            case 2: // Torque "0020 0060 000 0 00 00 0000 \0"
                strLength = "0020";
                strMid = "0060";
                strRevision = "001";
                break;
            case 3: // Torque Ack 0020 0062 001 0 00 00 0000 \0
                strLength = "0020";
                strMid = "0062";
                strRevision = "001";
                break;
            case 4: // Keep Alive "0020 9999 000 0 00 00 0000 \0"
                strLength = "0020";
                strMid = "9999";
                strRevision = "001";
                break;
            case 5: // PSet 0023 0018 001 0 00 00 0000 \0
                strLength = "0023";
                strMid = "0018";
                strRevision = "001";
                strAckFlag = "0";
                strStatinID = "    00";
                strPindleID = "  00";
                strSpare = ProgramNo;// string.Format("{0:000}", 1);
                break;
            case 6: // Tool Run 002302240010    00  007\0
                strLength = "0023";
                strMid = "0224";
                strRevision = "001";
                strAckFlag = "0";
                strStatinID = "    00";
                strPindleID = "  00";
                strSpare = "7";//string.Format("{0:000}", 7);
                break;
            case 7: // Tool Stop  002302240010    00  005\0
                strLength = "0023";
                strMid = "0224";
                strRevision = "001";
                strAckFlag = "0";
                strStatinID = "    00";
                strPindleID = "  00";
                strSpare = "5";//string.Format("{0:000}", 5);
                break;

            case 8: // Tool Reset   002302250010    00  005\0
                strLength = "0023";
                strMid = "0225";
                strRevision = "001";
                strAckFlag = "0";
                strStatinID = "    00";
                strPindleID = "  00";
                strSpare = "5";//string.Format("{0:000}", 5);
                break;

            case 9: // din 005 subscribe   002302200010    00  0000\0
                strLength = "0023";
                strMid = "0220";
                strRevision = "001";
                strAckFlag = "0";
                strStatinID = "    00";
                strPindleID = "  00";
                strSpare = "0";
                
            case 10: // din 005 upload ack   002302220010    00  0000\0
                strLength = "0023";
                strMid = "0222";
                strRevision = "001";
                strAckFlag = "0";
                strStatinID = "    00";
                strPindleID = "  00";
                strSpare = "0";
                break;
                
            case 11: //pset subcribe 00200014 001000  
                strLength = "0020";
                strMid = "0014";
                strRevision = "001";
                //strAckFlag = "0";
                //strStatinID = "    00";
                //strPindleID = "  00";
                //strSpare = "0";
                break;

            case 12: // pset ack 00200016 001000  .
                strLength = "0020";
                strMid = "0016";
                strRevision = "001";
                //strAckFlag = "0";
                //strStatinID = "    00";
                //strPindleID = "  00";
                //strSpare = "0";
                break;
            default:
                break;
        }

        sMsg = strLength + strMid + strRevision + strAckFlag + strStatinID + strPindleID + strSpare
                + strMsgEnd;
        return sMsg;
    }
	
	private void ProtocolParse(String data){
		//if(data.length() <= 0){
			//System.out.println("data Length error");
		//	return;
		//} 
		
		if(!data.substring(4,8).matches("9999") && data.length()>0 ){
			System.out.println("[CB: ]"+data.trim());
		}
		String Mid = data.substring(4, 8).trim();
		if(Mid.matches("0002")){
			//Start OK
			sendByte(MakeStr(2, "0").getBytes());
			sendByte(MakeStr(11, "0").getBytes());
		}
		else if(Mid.matches("0005")){
			//CMD Accept
			String aceeptID = data.substring(20, 380).trim();
			//System.out.println("[aceeptID: ]"+data.substring(20, 380).trim());
            if (aceeptID.matches("0018"))
            {
                //LogMsg.CMsg.Show(sName, "PSet Ok", "", false, true);
            	//System.out.println("[aceeptID: ] -->"+data.trim());
            }
            else if (aceeptID.matches("0060"))
            {
                //LogMsg.CMsg.Show(sName, "Torque subscript Ok", "", false, true);
            	//sendByte(MakeStr(9, "0").getBytes());
            }
            else if (aceeptID.matches("0224"))
            {
                //LogMsg.CMsg.Show(sName, "tool start or stop Ok", "", false, true);
            }
            else if (aceeptID.matches("0225"))
            {
                //LogMsg.CMsg.Show(sName, "tool reset Ok", "", false, true);
            }
            else if (aceeptID.matches("0220"))
            {
                //LogMsg.CMsg.Show(sName, "DIN subscript Ok", "", false, true);
               // sendByte(MakeStr(11, "0").getBytes());
            }
            else if (aceeptID.matches("0014"))
            {
                //LogMsg.CMsg.Show(sName, "PSet subscript Ok", "", false, true);
            }
		}
	    if(Mid.matches("0061")){
			//Toque Receive			
            float torque = Float.parseFloat(data.substring(142, 144) + "." + data.substring(144, 146));
            int tightState = Integer.parseInt(data.substring(107, 108));
            float angle = Float.parseFloat(data.substring(169, 174));
            icbReceive.cbToolResult(torque, tightState, angle);
			sendByte(MakeStr(3, "0").getBytes());
		}
		if(Mid.matches("0221")){
			//LogMsg.CMsg.Show(sName, "DIN Update Ok", "", false, true);
			System.out.println("DIN Update Ok " + data.substring(27, 28));
            int DINStatus = Integer.parseInt(data.substring(27, 28));
            icbReceive.cbToolDinStateResult(DINStatus);
            sendByte(MakeStr(10, "0").getBytes());
		}
		if(Mid.matches("0015")){
            int PsetNo = Integer.parseInt(data.substring(20, 23));
            icbReceive.cbToolPSetResult(PsetNo);
            sendByte(MakeStr(12, "0").getBytes());
		}
		if(Mid.matches("9999")){
			//KeepAlive ack DoNoting 
		}
	}
	
	public boolean HomePos() {
		int nTryCount = 0;
		int nMaxTryCount = 500;
		boolean check = false;
		int repeat = 1;
		onSendNutStop();

		//selectIO(NutInsert15N);
		onSendNutProgramSel(1);
		
		//ThreadUtil.milliSleep(300);

		for (int i = 0; i < 5; i++) {
			nTryCount =0;
			repeat = 1;
			onSendNutStart();
			while (repeat > 0) {
				nTryCount++;
				if (exio.getIN08()) {
					repeat = 0;
					check = true;
				} else {
					if (nTryCount > nMaxTryCount)
						repeat = 0;
				}
				ThreadUtil.milliSleep(10);
			}

			onSendNutStop();
			ThreadUtil.milliSleep(300);
			if (check) {
				if (exio.getIN08())
					break;
				else{
					check = false;
				}
			} 
		}
		if(check) System.out.println("nut runner home check Ok");
		else System.out.println("nut runner home check Ng");

		return check;
	}
	
	@Override
	public void run() {
		// TODO Auto-generated method stub
		startComm();
		
		//ArrayList<byte> msg = new ArrayList<>();
		byte[] msg = new byte[1024];
		//SndMsg_main = new byte[SEND_MSG_SIZE];
		System.out.println("Nut Thread Run");
		while (THREADFLAG) {
			try {
				if(!checkComm()){
					killComm();
					startComm();
					msg = new byte[1024];
					continue;
				}
				//logWrite("InWhile", false);
				msg = receiveByteWait(msg.length);
				/*if (msg == null) {
					if (thread_main) {
						//commOK_main = false;
						mSyncClient.endComm();
						mSyncClient.startComm();
						msg = new byte[RECV_MSG_SIZE];
						continue;
					}
				}*/
				//if(msg.length > 0){
				String buf = new String(msg);
				if(buf.trim().length() > 0){
					ProtocolParse(buf);
				}
				//}
		        ThreadUtil.milliSleep(10);
						
		        
			} catch (Exception ex) {

			}
		}
		System.out.println("<<<<< Nut Comm Kill>>>>>");
		if(checkComm()){
			killComm();
		}
		THREADFLAG = false;
	}
	
}
