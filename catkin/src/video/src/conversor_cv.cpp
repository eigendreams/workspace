#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/nonfree/nonfree.hpp"

//Este paso es solo para ahorrarse mas codigo despues
namespace enc = sensor_msgs::image_encodings;
using namespace cv;

//~ Aqui se crean las variables necesarias para el proceso
//~ de encontrar los puntos caracteristicos y mostrarlos
static const char WINDOW[] = "Image window";
static  int minHessian=400;
static  std::vector<KeyPoint> keypoints_1;

//~ La funcion detectora de puntos caracteristicos en este caso
//~ el SURF, se puede cambiar esta funcion 
static  SurfFeatureDetector detector( minHessian );

//~ En esta clase se hace todo el proceso, se utilizan
//~ los objetos de image_transport necesarios para suscribirse
//~ a la camara y para volver a publicar, el nodo de ros tambien
//~ se maneja aqui
class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  
public:
  ImageConverter()
    : it_(nh_)
  {
    image_pub_ = it_.advertise("out", 1);
    image_sub_ = it_.subscribe("in", 1, &ImageConverter::imageCb, this);
    cv::namedWindow(WINDOW);
    //~ Se inicializa la suscripcion a la camara y la publicacion de la imagen
    //~ con nombres "camara" y "out" respectivamente, con la informacion que
    //~ entra de la camara se llama a la funcion imageCb de esta misma clase.
  }

  ~ImageConverter()
  {
    cv::destroyWindow(WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
	  //~ Con ayuda de el puntero a la imagen del cv_bridge
	  //~ se pretende convertir la imagen que entra por la camara
	  //~ ahora llamada "msg" a una imagen que entienda opencv
    cv_bridge::CvImagePtr cv_ptr;
    
    //~ En el try-catch se prueba si la codificacion de entrada de la
    //~ camara puede ser copiada en formato BGR8, que es el que acepta
    //~ opencv, si no es capaz de convertirlo, marca un error
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    
    //~ Se corre el algoritmo detector de puntos caracteristicos y se guardan en el
    //~ vector keypoints_1. La imagen convertida de msg a formato opencv fue guardado
    //~ en cv_ptr desde el try-catch, el contenido de la imagen se convierte al tipo
    //~ MAT de opencv y queda guardado en el atributo cv_ptr->image
    detector.detect(cv_ptr->image,keypoints_1);
    
    //~ Se dibujan los puntos caracteristicos keypoints_1 en la imagen cv_ptr->image,
    //~ y se guarda de nuevo en cv_ptr->image
    drawKeypoints(cv_ptr->image,keypoints_1,cv_ptr->image,Scalar::all(-1),DrawMatchesFlags::DEFAULT);
    
    //~ Se muestra la imagen en una ventana de opencv y se espera un milisegundo
    cv::imshow(WINDOW, cv_ptr->image);
    cv::waitKey(1);
    
    //~ la misma imagen que se mostro, se publica en un mensaje de ros, para esto el objeto
    //~ cv_bridge::CvImagePtr tiene un metodo para convertir la imagen a el formato que
    //~ maneja ros, cv_ptr es un opbjeto del tipo CvImagePtr, por lo que la siguiente
    //~ expresion logra esa publicacion
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
	//~ En esta funcion main se inicializa el nodo de ros, al que llamamos image_converter,
	//~ se crea un objeto de la clase ImageConverter, en donde se maneja todo el 
	//~ algoritmo antes explicado, y por ultimo, se completa el loop
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
