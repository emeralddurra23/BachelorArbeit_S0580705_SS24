#ifndef MAIN_WINDOW_H
#define MAIN_WINDOW_H

#include <RecFusion.h>

#include <QtWidgets/QMainWindow>

#include <mutex>

class QLabel;
class QMessageBox;
class QTimer;

/** \brief	Main window of MultiViewReconstruction application
*/
class MainWindow : public QMainWindow
{
	Q_OBJECT

public:
	MainWindow();
	~MainWindow();

public slots:
	void processFrames();
	void calibrate();
	void performCalibration();
	void saveCalibration();
	void loadCalibration();
	void startReconstruction();
	void stopReconstruction();
	

private:
	std::vector<QLabel*> m_imgLabel;
	std::vector<std::vector<QLabel*>> m_recLabel;
	std::vector<QLabel*> m_reconstructedMeshLabels;
	QMessageBox* m_calibMessageBox;
	QTimer* m_timer;
	QImage renderMeshToImage(const RecFusion::Mesh& mesh);
	QImage renderMeshFromCamera(int cameraIndex, int toCameraIndex);
	std::vector<RecFusion::Sensor*> m_sensor;

	std::vector<RecFusion::ColorImage*> m_colorImg;
	std::vector<RecFusion::DepthImage*> m_depthImg;
	//std::vector<RecFusion::ColorImage*> m_sceneImg;
	std::vector<std::vector<RecFusion::ColorImage*>> m_sceneImg;
	std::vector<RecFusion::ColorImage*> m_calibImgColor;
	std::vector<RecFusion::DepthImage*> m_calibImgDepth;

	std::vector<RecFusion::Mat3> m_K;
	std::vector<RecFusion::Mat3> m_colorK;
	std::vector<RecFusion::Mat4> m_depthToColorT;
	std::vector<RecFusion::Mat4> m_sensorT;
	RecFusion::Mat4 computeRelativeTransform(int fromCameraIndex, int toCameraIndex);
	RecFusion::Vec3 transformPoint(const RecFusion::Mat4& transform, const RecFusion::Vec3& point);
	
	std::vector<bool> m_calibImgValid;

	bool m_reconstruct;
	bool m_calibrate;

	RecFusion::Reconstruction* m_rec;
	RecFusion::SensorManager* m_sensorManager;
	void postRefineMesh(RecFusion::Mesh& mesh);
	void applyTransformation(QImage& image, const RecFusion::Mat4& transform, const RecFusion::DepthImage* depthImage);
};

#endif

