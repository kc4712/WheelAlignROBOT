package additionFunction;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.net.UnknownHostException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.Timer;
import java.util.TimerTask;

import application.Auto_Adjustment;
import application.Manual_Robot;

import com.kuka.common.ThreadUtil;
import com.kuka.roboticsAPI.deviceModel.JointEnum;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.math.Vector;
import com.kuka.roboticsAPI.sensorModel.ForceSensorData;
import com.kuka.roboticsAPI.sensorModel.TorqueSensorData;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.task.ITaskLogger;

/**
 * @author Seulki-Kim , 2016. 4. 22. , KUKA Robotics Korea<p>
 * @modified Seulki-Kim , Apr 12, 2017 , KUKA Robotics Korea<p>
 * <pre>Usage example<br><code>
 * // Initialize
 * ForceTorqueDataSender FTrecorder = new ForceTorqueDataSender(
 * 	lbr, tcp, "172.31.1.101", 30000, -1, ForceTorqueDataSender.Type.FORCETORQUE_XYZABC,
 * 	"title" );
 *  // starting
 * FTrecorder.start();
 *  // motion
 * <i>motion to monitor F/T</i>
 *  // finishing
 * FTrecorder.interrupt();
 * </code></pre>
 */
@SuppressWarnings("unused")
public class ForceTorqueDataSender extends Thread {
	private LBR				lbr;
	private ObjectFrame		tcp;

	private long			time;
	private long			period;
	private double			tData;
	private int				count;

	private int				tcpPort;
	private UdpSender		udpServer;

	private Object			title;
	private String			_data;
	private ITaskLogger		logger;
	private int				packetCnt;
	private Type			type;

	private ForceSensorData		fsD;
	private TorqueSensorData	tqD;

	private Vector			fd;
	private Vector			td;
	private double[]		vals;
	private String			category;
	private double			startTime;

	/**<pre>
	 * <b>FORCETORQUE_XYZABC</b>	: Force XYZ [N] and Torque ABC [Nm]
	 * <b>JOINT_TORQUE</b>		: Joint torque J1~J7 [Nm]
	 * <b>BOTH</b>			: Both. ForceTorque first and then Joint Torque
	 </pre> */
	public enum Type {
		FORCETORQUE_XYZABC, JOINT_TORQUE, BOTH, TOE
	};

	/**<pre>
	 * @param lbr		LBR object
	 * @param tcp		TCP object - measure frame to calculate F/T
	 * @param tcpPort		Port - 30000 to 30010 are allowed
	 * @param time		auto finishing time in [ms]. use <b>-1</b> if you want to explicitly finish the class with interrupt()
	 * @param period		fixed-rate period - check every "period"[ms]
	 * @param type		Force&Torque / Joint Torque / Both
	 </pre> */
	public ForceTorqueDataSender(LBR lbr, ObjectFrame tcp, int tcpPort, long time, long period, Type type, ITaskLogger logger) {
		this.lbr = lbr;
		this.tcp = tcp;
		this.time = time<=0 ? (1000*60*60*24) : time;
		this.period = period;
		this.tData = 0;
		this._data = "";
		this.packetCnt = 0;
		this.tcpPort = tcpPort;
		this.type = type;
		this.title = null;
		
		//
		this.count = 0;
		
		this.logger = logger;
//		server = new TCPServer(tcpPort, logger);
//		server.startComm();
		udpServer = new UdpSender( "192.168.150.245", tcpPort );
	}
	/** <pre>
	 * @param lbr		LBR object
	 * @param tcp		TCP object - measure frame to calculate F/T
	 * @param tcpPort		Port - 30000 to 30010 are allowed
	 * @param time		auto finishing time in [ms]. use <b>-1</b> if you want to explicitly finish the class with interrupt()
	 * @param period		fixed-rate period - check every "period"[ms]
	 * @param type		Force&Torque / Joint Torque / Both
	 * @param title		title of a Data group. this will be the first packet to be sent
	 </pre> */
	public ForceTorqueDataSender(LBR lbr, ObjectFrame tcp, int tcpPort, long time, long period, Type type, ITaskLogger logger, String title) {
		this(lbr, tcp, tcpPort, time, period, type, logger);
		this.title = String.format(title + "\n");
	}
	
	/**
	 * @return	data from the start till now
	 */
	public String getData() {
		return _data;
	}
	
	/**
	 * @return	elapsed time [ms] from the start
	 */
	public double getTimeStamp() {
		return tData;
	}
	
	public void initialize() {
		category = null;
		if ( type == Type.FORCETORQUE_XYZABC ) {
			category = String.format("msec,X,Y,Z,TX,TY,TZ\n");
		} else if ( type == Type.JOINT_TORQUE ) {
			category = String.format("msec,J1,J2,J3,J4,J5,J6,J7\n");
		} else if ( type == Type.BOTH ) {
			category = String.format("msec,X,Y,Z,TX,TY,TZ, ,J1,J2,J3,J4,J5,J6,J7\n");				
		} else if ( type == Type.TOE ) {
			category = String.format("msec, LTOE, RTOE\n");				
		}
		

		startTime = System.nanoTime();
		count = 0;
		
	}
	
	@Override
	public void run() {
		initialize();
		
		// UDP cleint
		try {
			ScheduledJob job = new ScheduledJob();
			Timer jobScheduler = new Timer();
			jobScheduler.scheduleAtFixedRate(job, 0, period);

			while ( tData <= time ) {
//				loop();
//				keeping this run() alive while timer is working
//				timer is to be set '0' to get out of this while loop when this.interrupt() is called
			}
			// finish timer task
			jobScheduler.cancel();
			

			if ( time == 0 ) {
				System.out.println("Motion finished. Ending Data sending");
				System.out.println("Time taken : " + tData);
			} else {
				System.out.println("Data sending finished before motion is finished. ");
				
			}
			System.out.println("packets sent : " + packetCnt);

		} catch (Exception e) {
			e.printStackTrace();
		}	// end of try-catch for UDP


	}	// end of run()
	
	
	class ScheduledJob extends TimerTask {
		@Override
		public void run() {
			try {
				switch ( type ) {
				case FORCETORQUE_XYZABC:
					loopByteCartesian();
					break;
				case JOINT_TORQUE:
					loopByteJoint();
					break;
				case TOE:
					loopByteToe();
					break;
				
				}
			} catch (IOException e) {
				e.printStackTrace();
			}
		}

		
	}
	
	public void loopByteToe() {
		tData = ((double)System.nanoTime() - startTime)/1000000000d;
		count++;	// starting from 1
		
		tqD = lbr.getExternalTorque();
		
		byte[] data = new byte[32];

		float[] floatData = new float[8];		
		double test1 = 1.111;
		double test2 = 2.222;
		floatData[1] = Float.parseFloat( String.format("%03.02f", test1) );
		floatData[2] = Float.parseFloat( String.format("%03.02f", test2) );
		floatData[3] = Float.parseFloat( String.format("%03.02f", tqD.getSingleTorqueValue( JointEnum.J3 )) );
		floatData[4] = Float.parseFloat( String.format("%03.02f", tqD.getSingleTorqueValue( JointEnum.J4 )) );
		floatData[5] = Float.parseFloat( String.format("%03.02f", tqD.getSingleTorqueValue( JointEnum.J5 )) );
		floatData[6] = Float.parseFloat( String.format("%03.02f", tqD.getSingleTorqueValue( JointEnum.J6 )) );
		floatData[7] = Float.parseFloat( String.format("%03.02f", tqD.getSingleTorqueValue( JointEnum.J7 )) );

		byte[] bt = intToByteArray(count, ByteOrder.LITTLE_ENDIAN);
		byte[] b1 = floatToByteArray(floatData[1], ByteOrder.LITTLE_ENDIAN);
		byte[] b2 = floatToByteArray(floatData[2], ByteOrder.LITTLE_ENDIAN);
		byte[] b3 = floatToByteArray(floatData[3], ByteOrder.LITTLE_ENDIAN);
		byte[] b4 = floatToByteArray(floatData[4], ByteOrder.LITTLE_ENDIAN);
		byte[] b5 = floatToByteArray(floatData[5], ByteOrder.LITTLE_ENDIAN);
		byte[] b6 = floatToByteArray(floatData[6], ByteOrder.LITTLE_ENDIAN);
		byte[] b7 = floatToByteArray(floatData[7], ByteOrder.LITTLE_ENDIAN);

		for (int i = 0 ; i < 4; i++) {
			data[i] = bt[i];
			data[i+4] = b1[i];
			data[i+8] = b2[i];
			data[i+12] = b3[i];
			data[i+16] = b4[i];
			data[i+20] = b5[i];
			data[i+24] = b6[i];
			data[i+28] = b7[i];
		}
//		data[28] = 0x0D;	// CR
//		data[29] = 0x0A;	// LF

		_data = "";
		_data += String.format( floatData[0] +" "+ floatData[1] +" "+ floatData[2] +" "+
				floatData[3] +" "+ floatData[4] +" "+ floatData[5] +" "+ floatData[6] +" "+ floatData[7]);
		
		// send
		try {
//			server.sendByte(data);
//			server.send( _data )
			udpServer.sendString( _data );
		} catch (Exception e) {
			interrupt();
		}
		packetCnt++;
		
	}
	
	public void loopByteJoint() {
		tData = ((double)System.nanoTime() - startTime)/1000000000d;
		count++;	// starting from 1
		
		tqD = lbr.getExternalTorque();
		
		byte[] data = new byte[32];

		float[] floatData = new float[8];		
		floatData[1] = Float.parseFloat( String.format("%03.02f", tqD.getSingleTorqueValue( JointEnum.J1 )) );
		floatData[2] = Float.parseFloat( String.format("%03.02f", tqD.getSingleTorqueValue( JointEnum.J2 )) );
		floatData[3] = Float.parseFloat( String.format("%03.02f", tqD.getSingleTorqueValue( JointEnum.J3 )) );
		floatData[4] = Float.parseFloat( String.format("%03.02f", tqD.getSingleTorqueValue( JointEnum.J4 )) );
		floatData[5] = Float.parseFloat( String.format("%03.02f", tqD.getSingleTorqueValue( JointEnum.J5 )) );
		floatData[6] = Float.parseFloat( String.format("%03.02f", tqD.getSingleTorqueValue( JointEnum.J6 )) );
		floatData[7] = Float.parseFloat( String.format("%03.02f", tqD.getSingleTorqueValue( JointEnum.J7 )) );

		byte[] bt = intToByteArray(count, ByteOrder.LITTLE_ENDIAN);
		byte[] b1 = floatToByteArray(floatData[1], ByteOrder.LITTLE_ENDIAN);
		byte[] b2 = floatToByteArray(floatData[2], ByteOrder.LITTLE_ENDIAN);
		byte[] b3 = floatToByteArray(floatData[3], ByteOrder.LITTLE_ENDIAN);
		byte[] b4 = floatToByteArray(floatData[4], ByteOrder.LITTLE_ENDIAN);
		byte[] b5 = floatToByteArray(floatData[5], ByteOrder.LITTLE_ENDIAN);
		byte[] b6 = floatToByteArray(floatData[6], ByteOrder.LITTLE_ENDIAN);
		byte[] b7 = floatToByteArray(floatData[7], ByteOrder.LITTLE_ENDIAN);

		for (int i = 0 ; i < 4; i++) {
			data[i] = bt[i];
			data[i+4] = b1[i];
			data[i+8] = b2[i];
			data[i+12] = b3[i];
			data[i+16] = b4[i];
			data[i+20] = b5[i];
			data[i+24] = b6[i];
			data[i+28] = b7[i];
		}
//		data[28] = 0x0D;	// CR
//		data[29] = 0x0A;	// LF

		_data = "";
		_data += String.format( floatData[0] +" "+ floatData[1] +" "+ floatData[2] +" "+
				floatData[3] +" "+ floatData[4] +" "+ floatData[5] +" "+ floatData[6] +" "+ floatData[7] );
		
		// send
		try {
//			server.sendByte(data);
//			server.send( _data );
			udpServer.sendString( _data );
		} catch (Exception e) {
			interrupt();
		}
		packetCnt++;
		
	}
	
	public void loopByteCartesian() throws IOException {
		tData = ((double)System.nanoTime() - startTime)/1000000000d;
		count++;	// starting from 1
		
		fsD = lbr.getExternalForceTorque(tcp);
		fd = fsD.getForce();
		td = fsD.getTorque();
		
		byte[] data = new byte[28];

		float[] floatData = new float[7];
//		floatData[0] = Float.parseFloat( String.format("%03.02f", tData) );
//		floatData[0] = Integer.parseInt( String.format("%d", count));
		floatData[1] = Float.parseFloat( String.format("%03.02f", fd.getX()) );
		floatData[2] = Float.parseFloat( String.format("%03.02f", fd.getY()) );
		floatData[3] = Float.parseFloat( String.format("%03.02f", fd.getZ()) );
		floatData[4] = Float.parseFloat( String.format("%03.02f", td.getX()) );
		floatData[5] = Float.parseFloat( String.format("%03.02f", td.getY()) );
		floatData[6] = Float.parseFloat( String.format("%03.02f", td.getZ()) );

//		byte[] bt = floatToByteArray(floatData[0], ByteOrder.LITTLE_ENDIAN);
		byte[] bt = intToByteArray(count, ByteOrder.LITTLE_ENDIAN);
		byte[] bx = floatToByteArray(floatData[1], ByteOrder.LITTLE_ENDIAN);
		byte[] by = floatToByteArray(floatData[2], ByteOrder.LITTLE_ENDIAN);
		byte[] bz = floatToByteArray(floatData[3], ByteOrder.LITTLE_ENDIAN);
		byte[] btx = floatToByteArray(floatData[4], ByteOrder.LITTLE_ENDIAN);
		byte[] bty = floatToByteArray(floatData[5], ByteOrder.LITTLE_ENDIAN);
		byte[] btz = floatToByteArray(floatData[6], ByteOrder.LITTLE_ENDIAN);

		for (int i = 0 ; i < 4; i++) {
			data[i] = bt[i];
			data[i+4] = bx[i];
			data[i+8] = by[i];
			data[i+12] = bz[i];
			data[i+16] = btz[i];	// A
			data[i+20] = bty[i];	// B
			data[i+24] = btx[i];	// C
		}
//		data[28] = 0x0D;	// CR
//		data[29] = 0x0A;	// LF

		_data = "";
		_data += String.format( floatData[0] +" "+ floatData[1] +" "+ floatData[2] +" "+
				floatData[3] +" "+ floatData[4] +" "+ floatData[5] +" "+ floatData[6] );
		
		// send
		try {
//			server.sendByte(data);
//			server.send( _data );
			udpServer.sendString( _data );
		} catch (Exception e) {
			interrupt();
		}
		packetCnt++;
	}
	
	/**
	 * @param value		: byte[8] value
	 * @param byteOrder : ByteOrder.LITTLE_ENDIAN , ByteOrder.BIG_ENDIAN
	 * @return double value
	 */
	public double byte8ToDouble(byte[] value, ByteOrder byteOrder) {
		double ret = ByteBuffer.wrap(value).order(byteOrder).getDouble(0);
		return ret;
	}
	
	/**
	 * @param value		: Double value
	 * @param byteOrder : ByteOrder.LITTLE_ENDIAN , ByteOrder.BIG_ENDIAN
	 * @return byte[8] value
	 */
	public byte[] doubleToByteArray(double value, ByteOrder byteOrder) {
		return ByteBuffer.wrap(new byte[8]).order(byteOrder).putDouble(value).array();
	}
	
	public byte[] floatToByteArray(float value, ByteOrder byteOrder) {
		return ByteBuffer.wrap(new byte[4]).order(byteOrder).putFloat(value).array();
	}
	
	public byte[] intToByteArray(int value, ByteOrder byteOrder) {
		return ByteBuffer.wrap(new byte[4]).order(byteOrder).putInt(value).array();
	}
	
	@Override
	public void interrupt() {
//		super.interrupt();
		time = 0;
		System.out.println("Waiting for the thread to be finished...");
//		this.join(500);
		ThreadUtil.milliSleep(period*10);
//		server.endComm();
		udpServer.endComm();
	}	// end of interrupt()
	
}
