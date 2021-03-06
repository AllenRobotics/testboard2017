package org.usfirst.frc.team5417.robot;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.usfirst.frc.team5417.cv2017.ImageReader;
import org.usfirst.frc.team5417.cv2017.opencvops.ImageScaleOperation;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.VideoCamera;
import edu.wpi.first.wpilibj.CameraServer;

public class CameraReader implements ImageReader {

	private CvSink visionCameraSink;

	private Mat frame = new Mat();
	
	public CameraReader(CvSink cameraSink) {
		visionCameraSink = cameraSink;
	}

	@Override
	public Mat read() {
		visionCameraSink.grabFrame(frame);
		
		ImageScaleOperation initialScaleOp = new ImageScaleOperation(frame, 320);
		Mat resizedFrame = initialScaleOp.resize();
		frame.release();
		
		Mat f32f = new Mat();
		resizedFrame.assignTo(f32f, CvType.CV_32FC3);
		resizedFrame.release();
		
		return f32f;
	}
	
	public void free() {
		if (visionCameraSink != null) {
			visionCameraSink.free();
		}
	}

}
