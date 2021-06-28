#pragma once

#include <array>
#include <vector>
#include <map>
#include <unordered_map>
#include <string>
#include <fstream>
#include <limits>
#include <functional>
#define FMT_HEADER_ONLY
#include <fmt/format.h>
#include <cereal/cereal.hpp>
#include <cereal/types/array.hpp>
#include <cereal/types/vector.hpp>
#include <cereal/types/string.hpp>
#include <cereal/archives/binary.hpp>
#include <cereal/archives/json.hpp>
#include <pybind11/pybind11.h>
#include <glog/logging.h>

namespace vegas {

    namespace py = pybind11;

    using std::array;
    using std::vector;
    using std::map;
    using std::unordered_map;
    using std::string;
    using std::make_pair;
    using std::function;

    template <typename T>
    T get (py::dict dict, char const *name, T const &v) {
        if (dict.contains(name)) {
            return dict[name].cast<T>();
        }
        return v;
    }   

    // The linear space of 2-D points
    typedef array<double, 2> Point;

    static inline Point round (Point const &p) {
        return Point{std::round(p[0]), std::round(p[1])};
    }

    static inline Point operator + (Point const &p1, Point const &p2) {
        return Point{p1[0] + p2[0], p1[1] + p2[1]};
    }

    static inline Point operator - (Point const &p1, Point const &p2) {
        return Point{p1[0] - p2[0], p1[1] - p2[1]};
    }

    static inline Point operator + (Point const &p1, double v) {
        return Point{p1[0] + v, p1[1] + v};
    }

    static inline Point operator - (Point const &p1, double v) {
        return Point{p1[0] - v, p1[1] - v};
    }

    static inline Point operator * (Point const &p1, double v) {
        return Point{p1[0] * v, p1[1] * v};
    }

    static inline Point operator / (Point const &p1, double v) {
        return Point{p1[0] / v, p1[1] / v};
    }

    typedef array<Point, 2> Line;
    typedef vector<Point> Polygon;  // counter-clockwise arranged, 
                                    // same as opencv

    double area (Polygon const &, double scale = 1.0);
    void intersect (Polygon const &, Polygon const &, Polygon *);

    struct Shape {
        int code;               // type of shape
                                // code含义C++层面不管
                                // 上层保证0..n-1连续
        vector<Line> lines;     // all elements are converted
                                // to line segments
        template <class Archive>
        void serialize (Archive &ar)
        {
            ar(code);
            ar(lines);
        }
    };

    struct Box: public array<Point, 2> {

        Box () {
            (*this)[0][0] = (*this)[0][1] = std::numeric_limits<double>::max();
            (*this)[1][0] = (*this)[1][1] = -std::numeric_limits<double>::max();
        }

        Box (Point const &p, double relax = 0) {
            at(0) = p;
            at(1) = p;
            this->relax(relax);
        }

        Box (Point const &p1, Point const &p2, double relax = 0) {
            std::tie(at(0)[0], at(1)[0]) = std::minmax(p1[0], p2[0]);
            std::tie(at(0)[1], at(1)[1]) = std::minmax(p1[1], p2[1]);
            this->relax(relax);
        }

        Box (Polygon const &p): Box(p[0], p[2], 0) {
            // 输入的p必须是顺时针或者逆时针的直立矩形
            // p must contain 3 points
            // forming an upright box
            // 多边形必须是一个没有旋转过的矩形
            // 顺时针逆时针无所谓
            CHECK(p.size() == 4);
            if (p[0][0] == p[1][0]) {
                // p0 -- p1
                // |     |
                // p3 -- p2
                CHECK(p[2][0] == p[3][0]);
                CHECK(p[0][1] == p[3][1]);
                CHECK(p[1][1] == p[2][1]);
            }
            else {
                // p0 -- p3
                // |     |
                // p1 -- p2
                CHECK(p[0][0] == p[3][0]);
                CHECK(p[1][0] == p[2][0]);
                CHECK(p[0][1] == p[1][1]);
                CHECK(p[2][1] == p[3][1]);
            }
        }

        void relax (double r) {
            (*this)[0][0] -= r;
            (*this)[0][1] -= r;
            (*this)[1][0] += r;
            (*this)[1][1] += r;
        }


        Point center () const {
            return ((*this)[0] + (*this)[1]) / 2;
        }

        void contour (Polygon *c) const {
            c->clear();
            c->push_back(at(0));
            c->push_back(Point{at(1)[0], at(0)[1]});
            c->push_back(at(1));
            c->push_back(Point{at(0)[0], at(1)[1]});
        }

        double width () const {
            return std::max((*this)[1][0] - (*this)[0][0], 0.0);
        }

        double height () const {
            return std::max((*this)[1][1] - (*this)[0][1], 0.0);
        }
        
        double area () const {
            return width() * height();
        }

        bool empty () const {
            if ((*this)[0][0] >= (*this)[1][0]) return true;
            if ((*this)[0][1] >= (*this)[1][1]) return true;
            return false;
        }

        void extend (Point const &p) {
            (*this)[0][0] = std::min((*this)[0][0], p[0]);
            (*this)[0][1] = std::min((*this)[0][1], p[1]);
            (*this)[1][0] = std::max((*this)[1][0], p[0]);
            (*this)[1][1] = std::max((*this)[1][1], p[1]);
        }

        void extend_list (py::list p) {
            extend(Point{p[0].cast<double>(), p[1].cast<double>()});
        }

        void extend_tuple (py::tuple p) {
            extend(Point{p[0].cast<double>(), p[1].cast<double>()});
        }

        void extend_xy (py::object p) {
            extend(Point{p.attr("x").cast<double>(), p.attr("y").cast<double>()});

        }

        py::list unpack () const {
            // convert to list of [lbx, lby, ubx, uby]
            // lb = lower bound
            // ub = upper bound
            py::list v;
            v.append((*this)[0][0]);
            v.append((*this)[0][1]);
            v.append((*this)[1][0]);
            v.append((*this)[1][1]);
            return v;
        }

        bool contains (Point const &p) const {
            return (p[0] >= at(0)[0]) &&
                   (p[0] <= at(1)[0]) &&
                   (p[1] >= at(0)[1]) &&
                   (p[1] <= at(1)[1]);
        }

        bool contains (Line const &l) const {
            return contains(l[0]) && contains(l[1]);
        }

        bool contains (Shape const &sh) const {
            for (auto const &l: sh.lines) {
                if (!contains(l[0])) return false;
                if (!contains(l[1])) return false;
            }
            return true;
        }
    };

    static inline Box operator | (Box const &a, Box const &b) {
        Box c;
        c[0][0] = std::min(a[0][0], b[0][0]);
        c[0][1] = std::min(a[0][1], b[0][1]);
        c[1][0] = std::max(a[1][0], b[1][0]);
        c[1][1] = std::max(a[1][1], b[1][1]);
        return c;
    }

    static inline Box operator & (Box const a, Box const &b) {
        Box c;
        c[0][0] = std::max(a[0][0], b[0][0]);
        c[0][1] = std::max(a[0][1], b[0][1]);
        c[1][0] = std::min(a[1][0], b[1][0]);
        c[1][1] = std::min(a[1][1], b[1][1]);
        return c;
    }

    struct Layer {
        string name;
        vector<Shape> shapes; 

        template <class Archive>
        void serialize (Archive &ar)
        {
            ar(name);
            ar(shapes);
        }
    };

    // A CAD/vector graphics document is a list of layers
    struct Document {
        vector<Layer> layers;

        template <class Archive>
        void serialize (Archive &ar)
        {
            ar(layers);
        }

        void load (string const &path) {
            std::ifstream is(path, std::ios::binary);
            cereal::BinaryInputArchive archive(is);
            archive(*this);
        }

        void save (string const &path) {
            LOG(INFO) << "saving " << layers.size() << " layers.";

            std::ofstream os(path, std::ios::binary);
            cereal::BinaryOutputArchive archive(os);
            archive(*this);
        }

        int size () const {
            return layers.size();
        }

        string layerName (int l) const {
            return layers[l].name;
        }

        void renderLayer (py::object cvs, int layer) const {
            auto draw_line = cvs.attr("line");
            for (auto const &shape: layers[layer].shapes) {
                for (auto const &line: shape.lines) {
                    py::list p1, p2;
                    p1.append(line[0][0]);
                    p1.append(line[0][1]);
                    p2.append(line[1][0]);
                    p2.append(line[1][1]);
                    draw_line(p1, p2);
                }
            }
        }

        void render (py::object cvs, int layer) const {
            if (layer >= 0) {
                renderLayer(cvs, layer);
            }
            else {
                for (int i = 0; i < layers.size(); ++i) {
                    renderLayer(cvs, i);
                }
            }
        }
    };

    struct Markup {
        enum {
            CODE_ROI = 0,               // must be rect
            CODE_REMOVE,                // must be rect
            CODE_CONNECT,
            CODE_OBJECT_CONTOUR,        // 必须有label
            //CODE_OBJECT_BBOX            // 必须有label, must be rect
            // CONTOUR严格紧贴对象
            // BBOX会超出对象边界
        };
        int code;
        int label = -1;                 // 和layer的label是一致的
                                        // 如果label == -1则应用于所有label
        Polygon shape;

        template <class Archive>
        void serialize (Archive &ar)
        {
            ar(CEREAL_NVP(code), CEREAL_NVP(label), CEREAL_NVP(shape));
        }
    };

    // for object recognition
    struct Object {
        int label = -1;
        Box bbox;
        Polygon contour;
    };


    struct Detection {
        vector<Object> objects;
    };

    struct Annotation {
        vector<int> labels;         // layer labels
        vector<Markup> markups;     // by label

        template <class Archive>
        void serialize (Archive &ar)
        {
            ar(CEREAL_NVP(labels), CEREAL_NVP(markups));
        }

        void load_json (string const &path) {
            std::ifstream is(path);
            cereal::JSONInputArchive archive(is);
            serialize(archive);
        }

        void save_json (string const &path) {
            std::ofstream os(path);
            cereal::JSONOutputArchive archive(os);
            serialize(archive);
        }
    };

    class DocumentLoader: public Document {
        // work with vegas.DocumentLoadingCanvas
        map<string, int> lookup;
        int current;

        Point cast (py::object list) {
            double x = list[py::cast(0)].cast<double>();
            double y = list[py::cast(1)].cast<double>();
            return Point{x, y};
        }
    public:
        DocumentLoader (): current(-1) {
        }

        void select (string const &layer) {
            // select the current layer
            // create if layer does not exist
            auto r = lookup.insert(make_pair(layer, layers.size()));
            if (r.second) {
                layers.emplace_back();
                layers.back().name = layer;
            }
            current = r.first->second;
        }

        void add (int code, py::object list, bool closed) {
            layers[current].shapes.emplace_back();
            Shape &shape = layers[current].shapes.back();
            shape.code = code;
            int n = py::len(list);
            int l = n - 1;
            if (closed) l += 1;
            Point from = cast(list[py::cast(0)]);
            for (int i = 1; i <= l; ++i) {
                Point to = cast(list[py::cast(i % n)]);
                shape.lines.push_back(Line{from, to});
                from = to;
            }
        }
    };

    Box bound (Document const &);
    Box bound (Polygon const &);


    // 几何运算
    static inline double dot (Point const &p1, Point const &p2) {
        return p1[0] * p2[0] + p1[1] * p2[1];
    }

    static inline double det (Point const &p1, Point const &p2) {
        // det([p1; p2] as matrix)
        return p1[0] * p2[1] - p1[1] * p2[0];
    }
    
    static inline double norm (Point const &p) {
        return std::sqrt(dot(p, p));
    }

    static inline double norm (Line const &l) {
        return norm(l[1] - l[0]);
    }

    static inline double distance (Point const &p1, Point const &p2) {
        return norm(p2 - p1);
    }

    static inline Point direction (Point const &p1, Point const &p2) {
        Point p = p2 - p1;
        return p / norm(p);
    }

    static inline double distance (Line const &l, Point const &p) {
        Point dir = direction(l[0], l[1]);
        Point x = p - l[0];
        double a = norm(x);
        double b = dot(dir, x);
        return std::sqrt(a * a - b * b);
    }

    static inline Point intersect (Line const &a, Line const &b) {
        Point da = a[1] - a[0];
        Point db = b[1] - b[0];
        double u = det(a[0], da);
        double v = det(b[0], db);
        double bottom = det(db, da);
        CHECK(std::abs(bottom) > 1);
        double x = (u * db[0] - v * da[0]) / bottom;
        double y = (u * db[1] - v * da[1]) / bottom;
        return Point{x, y};
    }

    void extract_cc (Document const &doc, vector<Box> *bb, double cc_relax, int pick_layer = -1);

    template <typename T>
    class Factory {
    public:
        typedef map<string, function<T *(py::dict &)>> Registry;
        static Registry &registry () {
            static Registry impl;
            return impl;
        }
        static T *create (py::dict config) {
            string type = config["name"].cast<string>();
            auto &r = registry();
            auto it = r.find(type);
            if (it == r.end()) {
                std::cerr << type << " not found." << std::endl;
                CHECK(false);
            }
			return it->second(config);
        }
    };

	template <typename T>
	class Register {
    public:
        Register (string const &name) {
            T::registry()[name] = [](py::dict conf){ return new T(conf);};
        }
	};

#define REGISTER(cls)   Register<cls> register_##cls(#cls)

    class Extractor: public Factory<Extractor> {
    public:
        virtual size_t dim () const = 0;
        virtual void apply (Layer const &, double *ft) const = 0;
        virtual ~Extractor () {}
    };

    struct View {
        int label;
        vector<Layer const *> layers;
        vector<Markup const *> markups;
        View (Document const &doc, Annotation const &anno, int label_): label(label_) {
            CHECK(anno.labels.size() == doc.layers.size());
            for (unsigned i = 0; i < anno.labels.size(); ++i) {
                if (anno.labels[i] == label) {
                    layers.push_back(&doc.layers[i]);
                }
            }
            for (auto const &markup: anno.markups) {
                if (markup.label == -1 || markup.label == label) {
                    markups.push_back(&markup);
                }
            }
        }

        void collect (vector<Line> *) const;
        void collect_markup_objects (Detection *) const;
        void collect_markup_boxes (vector<Box> *boxes, int code) const {
            for (Markup const *m: markups) {
                if (m->code == code) {
                    boxes->emplace_back(m->shape);
                }
            }
        }
    };

    class Detector: public Factory<Detector> {
    public:
        virtual void detect (View const &, Detection *) const = 0;
        virtual ~Detector () {}
    };

    class ConnectedComponents {
        double relax;
        vector<Box> boxes;
    public:
        ConnectedComponents (double relax_): relax(relax_) {
        }

        void add (Line const &line) {
            Box r = Box(line[0], line[1], relax);
            int j = 0;
            while (j < boxes.size()) {
                auto one = boxes[j];
                if (!(one & r).empty()) {
                    r = r | one;
                    boxes[j] = boxes.back();
                    boxes.pop_back();
                }
                else {
                    ++j;
                }
            }
            boxes.push_back(r);
        }
        void cleanup () {
            for (;;) {
                int i = 0;
                int cc = 0;
                while (i < boxes.size()) {
                    auto &r = boxes[i];
                    int j = i+1;
                    while (j < boxes.size()) {
                        auto one = boxes[j];
                        if (!(one & r).empty()) {
                            r = r | one;
                            boxes[j] = boxes.back();
                            boxes.pop_back();
                            ++cc;
                        }
                        else {
                            ++j;
                        }
                    }
                    ++i;
                }
                if (cc == 0) break;
            }
        }

        void swap (vector<Box> *bb) {
            bb->swap(boxes);
        }
    };

}

