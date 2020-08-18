package additionFunction;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.net.UnknownHostException;
import java.util.LinkedList;
import java.util.Queue;
import java.util.logging.Logger;

/**
 * @author Seulki-Kim , Oct 18, 2016 , KUKA Robotics Korea<p>
 * usage example<p>
 * <pre><code>
 * UdpSender sender = new UdpSender("10.82.0.178", 30001);
 * sender.sendString(msg);
 * sender.endComm();
 * </code></pre>
 *
 */
@SuppressWarnings("unused")
public class UdpSender {
	private String			ip;
	private int				port;
	private DatagramSocket	ds;
	private InetAddress		adrs;
	private DatagramPacket	outPacket;
	private static Logger	logger;
	// thread send
	boolean					end_flag	= false;
	private Thread			rT;
	
	// Queue
	private Queue<String>	messageQueue;

	public UdpSender(String ip, int port) {
		this.ip = ip;
		this.port = port;
		
		try {
			adrs = InetAddress.getByName(ip);
			ds = new DatagramSocket();
				
		} catch (UnknownHostException e) {
			e.printStackTrace();
		} catch (SocketException e) {
			e.printStackTrace();
		}
		
		UdpSender.logger = Logger.getAnonymousLogger();
		logger.info("UDP sender opened");
		
		// start sendThread
		messageQueue = new LinkedList<String>();
		
		sendQueuedString rR = new sendQueuedString();
		rT = new Thread(rR);
		rT.start();
	}

	public void sendWithQueue(String msg) {
		messageQueue.add(msg);
	}

	public class sendQueuedString implements Runnable {
		@Override
		public void run() {
			while ( !Thread.currentThread().isInterrupted() && !end_flag ) {
				if ( !messageQueue.isEmpty() ) {
					//			for (int i=0; i< messageQueue.size()-1; i++) {
					sendString(messageQueue.poll());
				}	// end of if
			}	// end of for
		}	// end of run()
	}
	
	public void sendString(String msg) {
		try {
			byte[] buffer = msg.getBytes("UTF-8");
			outPacket = new DatagramPacket(buffer, buffer.length, adrs, port);
			ds.send(outPacket);

//			logger.info("[" + adrs + ":" + outPacket.getPort() + "] [send] : " + msg);								
			
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
	

	public void endComm() {
        logger.info("Ending UDP communication");
		ds.close();
		end_flag = true;
		try {
			rT.interrupt();
		} catch (Exception e) {
		}
	}
}
