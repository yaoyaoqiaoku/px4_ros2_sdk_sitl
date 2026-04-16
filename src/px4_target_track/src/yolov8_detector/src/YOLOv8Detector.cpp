#include "yolov8_detector/YOLOv8Detector.hpp"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <cstring>

namespace yolov8_detector {

// NumPy 初始化标志
static bool numpy_initialized = false;

// 初始化 NumPy API - 使用 import_array1 宏，它会处理返回值
static int init_numpy_api() {
    if (!numpy_initialized) {
        if (_import_array() < 0) {
            PyErr_Print();
            PyErr_SetString(PyExc_ImportError, "numpy.core.multiarray failed to import");
            return -1;
        }
        numpy_initialized = true;
    }
    return 0;
}

// 构造函数：加载模型，初始化Python环境。
YOLOv8Detector::YOLOv8Detector(const std::string& model_path,
                               float confidence_threshold,
                               float nms_threshold)
    : py_detector_(nullptr),    // Python端的YOLO模型对象指针
      confidence_threshold_(confidence_threshold),
      nms_threshold_(nms_threshold),
      input_size_(640, 640),
      next_track_id_(0)
{
    initialize_python();    // 确保Python解释器和NumPy已初始化
    
    // 初始化YOLOv8模型
    try {
        // --- 使用Python C API加载YOLOv8模型 ---
        // 1. 导入Python的“ultralytics”模块。
        // 2. 获取该模块中的“YOLO”类。
        // 3. 用提供的模型路径作为参数，实例化一个YOLO模型对象(`py_detector_`)。
        PyObject* pName = PyUnicode_DecodeFSDefault("ultralytics");
        PyObject* pModule = PyImport_Import(pName);
        Py_DECREF(pName);
        
        if (pModule == nullptr) {
            PyErr_Print();
            throw std::runtime_error("Failed to import ultralytics module");
        }
        
        // 获取YOLO类
        PyObject* pClass = PyObject_GetAttrString(pModule, "YOLO");
        Py_DECREF(pModule);
        
        if (pClass == nullptr) {
            PyErr_Print();
            throw std::runtime_error("Failed to get YOLO class");
        }
        
        // 创建模型实例
        PyObject* pModel = PyUnicode_FromString(model_path.c_str());
        if (pModel == nullptr) {
            Py_DECREF(pClass);
            throw std::runtime_error("Failed to create model path string");
        }
        
        PyObject* pArgs = PyTuple_Pack(1, pModel);
        Py_DECREF(pModel);
        
        if (pArgs == nullptr) {
            Py_DECREF(pClass);
            throw std::runtime_error("Failed to create arguments tuple");
        }
        
        py_detector_ = PyObject_CallObject(pClass, pArgs);
        Py_DECREF(pArgs);
        Py_DECREF(pClass);
        
        if (py_detector_ == nullptr) {
            PyErr_Print();
            throw std::runtime_error("Failed to create YOLO instance with model: " + model_path);
        }
        
        // 获取类别名称
        PyObject* pNames = PyObject_GetAttrString(static_cast<PyObject*>(py_detector_), "names");
        if (pNames != nullptr && PyDict_Check(pNames)) {
            PyObject* pKey = nullptr;
            PyObject* pValue = nullptr;
            Py_ssize_t pos = 0;
            
            int max_class_id = 0;
            while (PyDict_Next(pNames, &pos, &pKey, &pValue)) {
                int class_id = PyLong_AsLong(pKey);
                if (class_id > max_class_id) {
                    max_class_id = class_id;
                }
            }
            
            class_names_.resize(max_class_id + 1);
            pos = 0;
            while (PyDict_Next(pNames, &pos, &pKey, &pValue)) {
                int class_id = PyLong_AsLong(pKey);
                const char* class_name = PyUnicode_AsUTF8(pValue);
                if (class_name != nullptr && class_id >= 0 && class_id < static_cast<int>(class_names_.size())) {
                    class_names_[class_id] = class_name;
                }
            }
            Py_DECREF(pNames);
        }
        
        std::cout << "[YOLOv8Detector] Model loaded: " << model_path << std::endl;
        
    } catch (const std::exception& e) {
        cleanup_python();
        throw std::runtime_error(std::string("YOLOv8Detector initialization failed: ") + e.what());
    }
}

YOLOv8Detector::~YOLOv8Detector() {
    cleanup_python();
}

void YOLOv8Detector::initialize_python() {
    if (!Py_IsInitialized()) {
        Py_Initialize();
    }
    
    // 初始化NumPy API
    if (init_numpy_api() < 0) {
        throw std::runtime_error("Failed to initialize NumPy");
    }
}

void YOLOv8Detector::cleanup_python() {
    if (py_detector_ != nullptr) {
        Py_XDECREF(static_cast<PyObject*>(py_detector_));
        py_detector_ = nullptr;
    }
}

PyObject* YOLOv8Detector::cvmat_to_numpy(const cv::Mat& mat) {
    int depth = mat.depth();
    int channels = mat.channels();
    
    if (depth != CV_8U) {
        throw std::runtime_error("Unsupported image depth (only 8-bit supported)");
    }
    
    if (channels != 1 && channels != 3 && channels != 4) {
        throw std::runtime_error("Unsupported number of channels");
    }
    
    npy_intp shape[3];
    PyObject* arr = nullptr;
    int ndim = 0;
    
    if (channels == 1) {
        shape[0] = mat.rows;
        shape[1] = mat.cols;
        ndim = 2;
    } else {
        shape[0] = mat.rows;
        shape[1] = mat.cols;
        shape[2] = channels;
        ndim = 3;
    }
    
    arr = PyArray_SimpleNew(ndim, shape, NPY_UINT8);
    
    if (arr == nullptr) {
        throw std::runtime_error("Failed to create numpy array");
    }
    
    PyArrayObject* np_arr = reinterpret_cast<PyArrayObject*>(arr);
    void* arr_data = PyArray_DATA(np_arr);
    size_t total_size = mat.total() * mat.channels();
    std::memcpy(arr_data, mat.data, total_size);
    
    if (channels == 3) {
        PyObject* globals = PyDict_New();
        PyObject* locals = PyDict_New();
        PyDict_SetItemString(globals, "arr", arr);
        
        PyObject* arr_rgb = PyRun_String(
            "arr[..., ::-1].copy()", 
            Py_eval_input, 
            globals, 
            locals
        );
        
        Py_DECREF(globals);
        Py_DECREF(locals);
        Py_DECREF(arr);
        
        if (arr_rgb == nullptr) {
            PyErr_Print();
            throw std::runtime_error("Failed to convert BGR to RGB");
        }
        
        return arr_rgb;
    }
    
    return arr;
}

void YOLOv8Detector::detect(const cv::Mat& image,
                            std::vector<DetectionResult>& results,
                            int target_class)
{
    results.clear();
    
    if (py_detector_ == nullptr || image.empty()) {
        return;
    }
    
    try {
        PyObject* pImage = cvmat_to_numpy(image);
        if (pImage == nullptr) {
            throw std::runtime_error("Failed to convert image to numpy array");
        }
        
        PyObject* pPredict = PyObject_GetAttrString(static_cast<PyObject*>(py_detector_), "predict");
        if (pPredict == nullptr) {
            Py_DECREF(pImage);
            PyErr_Print();
            throw std::runtime_error("Failed to get predict method");
        }
        
        PyObject* pArgs = PyTuple_Pack(1, pImage);
        PyObject* pKwargs = PyDict_New();
        
        if (pKwargs != nullptr) {
            PyDict_SetItemString(pKwargs, "conf", PyFloat_FromDouble(confidence_threshold_));
            PyDict_SetItemString(pKwargs, "iou", PyFloat_FromDouble(nms_threshold_));
            PyDict_SetItemString(pKwargs, "verbose", Py_False);
        }
        
        PyObject* pResults = PyObject_Call(pPredict, pArgs, pKwargs);
        Py_DECREF(pPredict);
        Py_DECREF(pArgs);
        Py_XDECREF(pKwargs);
        Py_DECREF(pImage);
        
        if (pResults == nullptr) {
            PyErr_Print();
            throw std::runtime_error("Predict failed");
        }
        
        Py_ssize_t num_results = PyList_Size(pResults);
        for (Py_ssize_t i = 0; i < num_results; ++i) {
            PyObject* pResult = PyList_GetItem(pResults, i);
            if (pResult == nullptr) continue;
            
            PyObject* pBoxes = PyObject_GetAttrString(pResult, "boxes");
            if (pBoxes == nullptr) continue;
            
            PyObject* pXYXY = PyObject_GetAttrString(pBoxes, "xyxy");
            PyObject* pConf = PyObject_GetAttrString(pBoxes, "conf");
            PyObject* pCls = PyObject_GetAttrString(pBoxes, "cls");
            
            if (pXYXY != nullptr && pConf != nullptr && pCls != nullptr) {
                Py_ssize_t num_boxes = PyObject_Length(pXYXY);
                
                for (Py_ssize_t j = 0; j < num_boxes; ++j) {
                    // 创建索引对象
                    PyObject* pIndex = PyLong_FromLong(j);
                    
                    PyObject* pBox = PyObject_GetItem(pXYXY, pIndex);
                    PyObject* pConf_j = PyObject_GetItem(pConf, pIndex);
                    PyObject* pCls_j = PyObject_GetItem(pCls, pIndex);
                    
                    Py_DECREF(pIndex);
                    
                    if (pBox != nullptr && pConf_j != nullptr && pCls_j != nullptr) {
                        // 获取bbox坐标
                        PyObject* pIdx0 = PyLong_FromLong(0);
                        PyObject* pIdx1 = PyLong_FromLong(1);
                        PyObject* pIdx2 = PyLong_FromLong(2);
                        PyObject* pIdx3 = PyLong_FromLong(3);
                        
                        PyObject* pCoord0 = PyObject_GetItem(pBox, pIdx0);
                        PyObject* pCoord1 = PyObject_GetItem(pBox, pIdx1);
                        PyObject* pCoord2 = PyObject_GetItem(pBox, pIdx2);
                        PyObject* pCoord3 = PyObject_GetItem(pBox, pIdx3);
                        
                        float x1 = (pCoord0 != nullptr) ? PyFloat_AsDouble(pCoord0) : 0.0f;
                        float y1 = (pCoord1 != nullptr) ? PyFloat_AsDouble(pCoord1) : 0.0f;
                        float x2 = (pCoord2 != nullptr) ? PyFloat_AsDouble(pCoord2) : 0.0f;
                        float y2 = (pCoord3 != nullptr) ? PyFloat_AsDouble(pCoord3) : 0.0f;
                        
                        Py_DECREF(pIdx0);
                        Py_DECREF(pIdx1);
                        Py_DECREF(pIdx2);
                        Py_DECREF(pIdx3);
                        Py_XDECREF(pCoord0);
                        Py_XDECREF(pCoord1);
                        Py_XDECREF(pCoord2);
                        Py_XDECREF(pCoord3);
                        
                        float conf = PyFloat_AsDouble(pConf_j);
                        int class_id = PyLong_AsLong(pCls_j);
                        
                        if (target_class >= 0 && class_id != target_class) {
                            Py_DECREF(pBox);
                            Py_DECREF(pConf_j);
                            Py_DECREF(pCls_j);
                            continue;
                        }
                        
                        DetectionResult det;
                        det.class_id = class_id;
                        det.class_name = (class_id >= 0 && class_id < static_cast<int>(class_names_.size())) 
                                        ? class_names_[class_id] : "unknown";
                        det.confidence = conf;
                        det.bbox = cv::Rect2f(x1, y1, x2 - x1, y2 - y1);
                        det.center = cv::Point2f((x1 + x2) / 2, (y1 + y2) / 2);
                        det.track_id = -1;
                        
                        results.push_back(det);
                    }
                    
                    Py_XDECREF(pBox);
                    Py_XDECREF(pConf_j);
                    Py_XDECREF(pCls_j);
                }
            }
            
            Py_XDECREF(pXYXY);
            Py_XDECREF(pConf);
            Py_XDECREF(pCls);
            Py_XDECREF(pBoxes);
        }
        
        Py_DECREF(pResults);
        
    } catch (const std::exception& e) {
        std::cerr << "[YOLOv8Detector] Detection error: " << e.what() << std::endl;
    }
}

void YOLOv8Detector::update_tracking(const std::vector<DetectionResult>& current_detections,
                                     std::vector<TrackingTarget>& tracked_targets)
{
    tracked_targets = tracked_targets_;
    
    const float distance_threshold = 50.0f;
    std::vector<bool> detection_matched(current_detections.size(), false);
    std::vector<bool> track_matched(tracked_targets_.size(), false);
    
    for (size_t i = 0; i < tracked_targets_.size(); ++i) {
        float min_distance = distance_threshold;
        int best_match = -1;
        
        for (size_t j = 0; j < current_detections.size(); ++j) {
            if (detection_matched[j]) continue;
            
            float dx = tracked_targets_[i].center.x - current_detections[j].center.x;
            float dy = tracked_targets_[i].center.y - current_detections[j].center.y;
            float distance = std::sqrt(dx * dx + dy * dy);
            
            if (distance < min_distance) {
                min_distance = distance;
                best_match = j;
            }
        }
        
        if (best_match >= 0) {
            tracked_targets_[i].center = current_detections[best_match].center;
            tracked_targets_[i].bbox = current_detections[best_match].bbox;
            tracked_targets_[i].confidence = current_detections[best_match].confidence;
            tracked_targets_[i].frames_without_detection = 0;
            detection_matched[best_match] = true;
            track_matched[i] = true;
        } else {
            tracked_targets_[i].frames_without_detection++;
        }
    }
    
    auto it = tracked_targets_.begin();
    while (it != tracked_targets_.end()) {
        if (it->frames_without_detection > 10) {
            it = tracked_targets_.erase(it);
        } else {
            ++it;
        }
    }
    
    for (size_t j = 0; j < current_detections.size(); ++j) {
        if (!detection_matched[j]) {
            TrackingTarget new_track;
            new_track.track_id = next_track_id_++;
            new_track.class_id = current_detections[j].class_id;
            new_track.class_name = current_detections[j].class_name;
            new_track.center = current_detections[j].center;
            new_track.bbox = current_detections[j].bbox;
            new_track.confidence = current_detections[j].confidence;
            new_track.frames_without_detection = 0;
            new_track.velocity = cv::Point2f(0, 0);
            
            tracked_targets_.push_back(new_track);
        }
    }
    
    // 更新轨迹信息
    for (const auto& target : tracked_targets_) {
        auto& trail = track_trails_[target.track_id];
        trail.push_back(target.center);
        
        // 限制轨迹长度为最近10个位置
        if (trail.size() > 10) {
            trail.pop_front();
        }
    }
    
    // 清除已删除目标的轨迹
    std::vector<int> ids_to_remove;
    for (auto& kv : track_trails_) {
        bool found = false;
        for (const auto& target : tracked_targets_) {
            if (target.track_id == kv.first) {
                found = true;
                break;
            }
        }
        if (!found) {
            ids_to_remove.push_back(kv.first);
        }
    }
    
    for (int id : ids_to_remove) {
        track_trails_.erase(id);
    }
    
    tracked_targets = tracked_targets_;
}

std::deque<cv::Point2f> YOLOv8Detector::get_track_trail(int track_id) const {
    auto it = track_trails_.find(track_id);
    if (it != track_trails_.end()) {
        return it->second;
    }
    return std::deque<cv::Point2f>();
}

}  // namespace yolov8_detector