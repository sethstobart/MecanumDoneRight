package org.usfirst.frc.team4611.robot;

import java.io.IOException;
import java.text.SimpleDateFormat;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.logging.ConsoleHandler;
import java.util.logging.FileHandler;
import java.util.logging.Formatter;
import java.util.logging.Handler;
import java.util.logging.Level;
import java.util.logging.LogRecord;
import java.util.logging.Logger;

public class OzoneJavaLogger {
	
	private static OzoneJavaLogger instance	= null;
	private Logger logger	= Logger.getLogger("org.usfirst.frc.team4611.robot");
	private boolean	initted	= false;

	private OzoneJavaLogger() {
		
	}
	
	public static OzoneJavaLogger getInstance() {
		if (instance == null) {
			instance	= new OzoneJavaLogger();
		}
		return instance;
	}

	public void init(Level logLevel) {
		if (initted)
				return;
		
		logger.setUseParentHandlers(false);
		
		try {
			logger.setLevel(logLevel);
			
			Handler	consoleHandler	= new ConsoleHandler();
			consoleHandler.setFormatter(new OzoneLogFormatter());
			consoleHandler.setLevel(Level.SEVERE);
			logger.addHandler(consoleHandler);
			
			Handler fileHandler = new FileHandler("PlayLogger-%g",
					20*1024*1024,
					10,
					false);
			
			fileHandler.setFormatter(new OzoneLogFormatter());
			fileHandler.setLevel(Level.FINEST);
			logger.addHandler(fileHandler);
			
		} catch (SecurityException e) {
			logger.log(Level.SEVERE, "XX", e);
		} catch (IOException e) {
			logger.log(Level.SEVERE, "XX", e);
		} finally {
			initted	= true;
		}
	}
	
	
	public class OzoneLogFormatter extends Formatter {
		StringBuffer	b	= new StringBuffer();
		SimpleDateFormat dateFormat	= new SimpleDateFormat("HH.mm:ss");

		@Override
		public String format(LogRecord record)  {
			b.setLength(0);
		
			b.append("[" + record.getLevel()).append("]")
				.append('[').append(LocalDateTime.now().format(DateTimeFormatter.ofPattern("HH:mm:ss.SSS"))).append(']')
				.append('[').append(record.getSourceClassName()).append(']')
				.append('[').append(record.getMessage()).append(']')
				.append('\n');
			return b.toString();
		}		
	}
	
	public class LogTest {
		final Logger	logger	= Logger.getLogger(LogTest.class.getName());
		public LogTest() {
			
		}
		
		public void test() {
			logger.info("LogTest info");
			logger.fine("LogTest fine");
		}
	}

	public static void main(String[] args) {
		
		OzoneJavaLogger.getInstance().init(Level.FINE);
		
		Logger logger	= Logger.getLogger("org.usfirst.frc.team4611.robot.main");
		logger.info("Information");
		logger.severe("Severe");
		LogTest logTest	=  OzoneJavaLogger.getInstance().new LogTest();
		logTest.test();
		
		logger.finest("main finest");
	}

}
