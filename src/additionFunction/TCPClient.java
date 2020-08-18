package additionFunction;

import java.io.BufferedInputStream;
import java.io.BufferedOutputStream;
import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.io.OutputStreamWriter;
import java.io.PrintWriter;
import java.net.InetAddress;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.SocketException;
import java.nio.CharBuffer;


/**
 * @author Seulki-Kim usage e.g)<br>
 *         <code>
 * TCPClient  client;	// 선언<br>
 * 
 * client = new TCPClient("192.168.1.7");	// 초기화 param:server ip<br>
 * 
 * client.startComm();	// Server 로 연결 시도<br>
 * client.send("message")	// "message" 전송<br>
 * String msg = client.receiveWait();	// server 로부터 메시지 전송받아 msg 변수에 저장<br>
 * 
 * client.endComm();	// 연결 종료<br>
 * </code>
 */
public class TCPClient {
    private Thread rT;
    private Socket client_socket;
    private Socket socket;
    private static BufferedReader in;
    private static PrintWriter out;
	boolean end_flag = false;
	private String ip;

	static int port;
    String threadedMessage;
	public byte[] carInfos;
    public String getThreadedMessage() {
		return threadedMessage;
	}

    
    public TCPClient (String ip, int nPort) {
    	port = nPort;
    	this.ip = ip;
    }
    
	/**
	 * @param message : message to send to client
	 */
	public void send(String message) {
        try {
			System.out.println("TCP Server [SEND] : \"" + message + "\"");
			out.println(message);   // sending message via outputstream
		} catch (Exception e) {
			e.printStackTrace();
		}
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
	
	public void startComm() {
    	System.out.println("Starting TCPCliet communication");
   	 try{
        	client_socket = null;
			int maxretry = 100;
			
			while ( maxretry-- > 0 ) {
				try {
					client_socket = new Socket(InetAddress.getByName(ip), port);
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


        } catch (IOException e) {
            System.out.println("TCPClient Server : Error" + e);
        } catch(Exception ex) {
        	
        }
    	 
	}
	
	public void endComm() {
        try {
        	System.out.println("Ending TCP communication");
			socket.close();
			client_socket.close();
			end_flag = true;
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
}
