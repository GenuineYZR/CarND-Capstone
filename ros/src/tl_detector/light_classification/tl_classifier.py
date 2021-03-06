from styx_msgs.msg import TrafficLight
import os
import tensorflow as tf
import numpy as np
from utilities import label_map_util

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        self.current_light = TrafficLight.UNKNOWN

        cwd = os.path.dirname(os.path.realpath(__file__)) # Current working directory
        self.simulation = True

        if self.simulation is True:
            CKPT = cwd + '/graphs/simulator_graph.pb'
        else:
            CKPT = cwd + '/graphs/test_site_graph.pb'

        PATH_TO_LABELS = cwd+'/graphs/label_map.pbtxt'
        NUM_CLASSES = 4

        label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
        categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
        self.category_index = label_map_util.create_category_index(categories)

        ##### Build network
        self.image_np_deep = None
        self.detection_graph = tf.Graph()

        # https://github.com/tensorflow/tensorflow/issues/6698
        config = tf.ConfigProto()
        config.gpu_options.allow_growth = True
        # end

        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()

            with tf.gfile.GFile(CKPT, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

            self.sess = tf.Session(graph=self.detection_graph, config=config)

         # Definite input and output Tensors for detection_graph
        self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')

        # Each box represents a part of the image where a particular object was detected.
        self.detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')

        # Each score represent how level of confidence for each of the objects.
        # Score is shown on the result image, together with the class label.
        self.detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
        self.detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
        self.num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')

        print("Detection graph loaded")


    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        run_network = True  # flag to disable running network if desired

        if run_network is True:
            image_np_expanded = np.expand_dims(image, axis=0)

            # Actual detection.
            with self.detection_graph.as_default():
                (boxes, scores, classes, num) = self.sess.run(
                    [self.detection_boxes, self.detection_scores, 
                    self.detection_classes, self.num_detections],
                    feed_dict={self.image_tensor: image_np_expanded})

            boxes = np.squeeze(boxes)
            scores = np.squeeze(scores)
            classes = np.squeeze(classes).astype(np.int32)

            min_score_thresh = .50
            for i in range(boxes.shape[0]):
                if scores is None or scores[i] > min_score_thresh:
                    
                    class_name = self.category_index[classes[i]]['name']
                    # class_id = self.category_index[classes[i]]['id']  # if needed

                    print('{}'.format(class_name))

                    # Traffic light thing
                    self.current_light = TrafficLight.UNKNOWN

                    if class_name == 'Red':
                        self.current_light = TrafficLight.RED
                        # print('Red') 
                    elif class_name == 'Green':
                        self.current_light = TrafficLight.GREEN
                        # print('Green')
                    elif class_name == 'Yellow':
                        self.current_light = TrafficLight.YELLOW
                        # print('Yellow')

        return self.current_light
