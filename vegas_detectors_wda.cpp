#include "vegas.h"

namespace vegas { namespace detectors {

//    static const int CC_RELAX = 5000;
//    static const int OBJECT_RELAX = 10; // 1cm
//    static const int OBJECT_ROOM_TEST_RELAX = 500;
//    static const int ROOM_SIZE_MAX_THRESHOLD = 20000;
//    static const int ROOM_SIZE_MIN_THRESHOLD = 15000;
//    static const int CONNECT_TH = 8000; // 2米
//    static const int CONNECT_TH_TINY = 500; // 2米
//    static const int CONNECT_TH_SMALL = 2000; // 2米
//    static const int CONNECT_TH_MEDIUM = 8000; // 8米   sample-5xy
//    static const int CONNECT_TH_LONG = 10000; // 8米
//    static const int MERGE_GAP_TH = 50;
//    static const float PARALLEL_TH = 0.5;
//    static const int OPEN_KERNEL = 5;
//    static const int RANK_GROUP_TH = 10000;
//    static const int DOOR_EXIT_THRESHOLD = 2800;
//    static const int EXTEND_LINE_THRESHOLD = 1000;
//    static const int EXTEND_LINE = 2000;
//    static const int FIRE_HYDRANT_PILLAR_GAP = 500;
//    static const int FIRE_ACC_GUARD_SIZE = 1500;
//    static const int FIRE_GUARD_2M_SIZE = 1000;
//    static const int ATTRACT_ASPECT_RATIO = 20;
//    static const int ATTRACT_TOLERATE = 5;
//    static const int MAX_NON_REAL_EXTEND = 1;
//    static const int MIN_DUAL_EXTEND = 2000;
//    static const float POPULAR_VALUE_TH = 0.3;
//    static const int POPULAR_TH = 3;
//    static const int ALIGN_TO_ROOM_RELAX = 1500;
//    static const int WALL_GUARDS_DIST = 500;
//    static const int WALL_GUARDS_TRIANGULATE = 200;

    struct XLine: Line {
        Line core;
        bool real;    // 点是在实际结构上的
        float length; // length of line, not core
        float angle;  // digree, so we can do exact match
                      // 0 - 179
        bool update1 = false;
        bool update2 = false;

        XLine () {}

        XLine (Point p1, Point p2, bool r = true): real(r) {
            Point d = p2 - p1;              // angle: [-pi : pi]
            if (d[1] == 0) {
                CHECK(d[0] != 0);
                if (p1[0] > p2[1]) {          // p1 -> p2
                                            // p2.x is bigger
                    std::swap(p1, p2);
                }
                angle = 0;
            }
            else {
                if (d[1] < 0) {  
                    std::swap(p1, p2);
                    d = p2 - p1;
                }                               // angle: [0: pi]
                // p2 on top of p1
                if (d[0] == 0) {
                    angle = 90;
                }
                else {
                    double r = std::atan2(d[1], d[0]);
                    angle = 180 * r / M_PI;
                    CHECK(angle >= 0);
                    CHECK(angle < 180);
                }
            }
            at(0) = p1;
            at(1) = p2;
            core[0] = p1;
            core[1] = p2;
            update_length();
        }

        explicit XLine (Line const &ln, bool r = true): XLine(ln[0], ln[1], r) {}

        void update_length () {
            length = norm(*self):
        }

        // 长度
        double norm () const {
            return length;
        }
    };

    bool parallel (XLine const &l1, XLine const &l2, double th) {
        float a = l1.angle;
        float b = l2.angle;
        if (a > b) std::swap(a, b);
        // a <= b
        float d = std::min(b - a, a+180-b);
        return d <= th;
	}

    bool connect_parallel (XLine l1, XLine l2, int th, XLine *conn) {
        if (!parallel(l1, l2)) return false;
        // try connect
        double n1 = l1.norm();
        double n2 = l2.norm();
        if (n1 < n2) {
            std::swap(l1, l2);
            std::swap(n1, n2);
        }
        // l1 is longer than (or equal to) l2
        double best = -1;
        int best1 = -1;
        int best2 = -1;
        for (int i = 0; i < 2; ++i) {
            for (int j = 0; j < 2; ++j) {
                double n = norm(l1[i], l2[j]);
                if (n > best) {
                    best = n;
                    best1 = i;
                    best2 = j;
                }
            }
        }
        CHECK(best1 >= 0 && best2 >= 0);
        if (best <= n1) {   // l1 covers l2, succeeed
            if (!(distance(l1, l2[0]) < MERGE_GAP_TH)) return false;
            if (!(distance(l1, l2[1]) < MERGE_GAP_TH)) return false;
            *conn = l1;
            return true;
        }
        if (best < 1) return false; // connected line is too short
        // now the outer points are l1[0] and l2[1]
        Line c{l1[best1], l2[best2]};
        XLine cx(c); // direction of cx might change, that's why we need c
        // the connected line must be parallel to both components
        if (!parallel(cx, l1)) return false;
        if (!parallel(cx, l2)) return false;
        // inner points are l1[1] and l2[0]
        cv::Point2d dir(direction(c[0], c[1]));
        double o1 = dot(dir, l1[1-best1] - c[0]); // , n1, l1.norm());
        double o2 = dot(dir, l2[1-best2] - c[0]);
        if (std::abs(o2 - o1) > th) return false;
        *conn = cx;
        return true;
    }

    bool connect_strict_parallel (XLine l1, XLine l2, int th, XLine *conn) {
        cv::Point2d v1 = l1[1] - l1[0];
        cv::Point2d v2 = l2[1] - l2[0];
        double cross = v1.x * v2.y - v2.x * v1.y;
        if (cross != 0) return false;
        return connect_parallel(l1, l2, th, conn);
    }


    struct Extend {
        // 在线段所在的直线上找一个点
        // 不管这个点在哪儿，都是可以试图延长这个线段直到这个点的

        // hit表明点的距离是否足够近
        bool hit = false;   
        // side表明点是否在延长线上，而不在线段内部
        int side = -1;
                    // -1 not hit
                    // 0 extend point 0
                    // 1 extend point 1
                    // 2 inside, do not extend
        cv::Point pt;
        bool real = true;

        // 如果pt在线段之外则硬延长返回true
        // 否则返回false, 和hit无关
        bool apply (XLine *line) const {
            //if (!hit) return false;
            if (side == 0) {
                line->at(0) = pt;
                if (real) line->core[0] = pt;
                line->update_length();
                return true;
            }
            else if (side == 1) {
                line->at(1) = pt;
                if (real) line->core[1] = pt;
                line->update_length();
                return true;
            }
            return false;
        }
    };

    Extend try_extend (XLine const &line, cv::Point2d const &pt, bool real, int th, int tip = 3) {
        // real 表示点pt是在实际结构上的
        cv::Point2d dir(line.core[1]-line.core[0]);
        double l = norm(dir);
        dir /= l;

        double lb = dot(dir, cv::Point2d(line[0]) - cv::Point2d(line.core[0]));
        double rb = dot(dir, cv::Point2d(line[1]) - cv::Point2d(line.core[0]));

        double x = dot(dir, cv::Point2d(pt - cv::Point2d(line.core[0])));

        Extend ext;
        ext.real = real;
        if (x < lb) {
            if (x >= -th) {
                ext.hit = true;
            }
            x -= tip;
            ext.side = 0;
            cv::Point2d delta = x * dir;
            ext.pt = cv::Point(round(line.core[0].x + delta.x),
                               round(line.core[0].y + delta.y));
        }
        else if (x > rb) {
            if (x <= l + th) {
                ext.hit = true;
            }
            x += tip;
            ext.side = 1;
            x -= l;
            cv::Point2d delta = x * dir;
            ext.pt = cv::Point(round(line.core[1].x + delta.x),
                               round(line.core[1].y + delta.y));
        }
        else if ((x >= lb) && (x <= rb)) {
            ext.hit = true;
            ext.side = 2;
        }
        return ext;
    }


    void round_to_group_lb (vector<int> &v) {
        /*
    # 先对输入的数值进行分组
    # 分组方法为：所有数值排序，然后当相邻两个数字之间大于GROUPING_TH是切断
    # 按切断方法对原数组进行分组
    # 然后把原数组所有的数值替换为其所在组最小的值

    # 目的是让一群大致有序的数字，其中差不多的数值变成一样
        */
        vector<pair<int, int>> o;
        for (int i = 0; i < v.size(); ++i) {
            o.push_back(make_pair(v[i], i));
        }
        sort(o.begin(), o.end());
        int begin = 0;
        while (begin < o.size()) {
            int end = begin + 1;
            while (end < o.size() && (o[end].first - o[end-1].first) < RANK_GROUP_TH) {
                ++end;
            }
            for (int i = begin; i < end; ++i) {
                v[o[i].second] = o[begin].first;
            }
            begin = end;
        }
    }

    void sort_rooms (vector<Object> const &rooms, vector<int> *order) {
        vector<int> xs;
        vector<int> ys;
        for (auto const &r: rooms) {
            xs.push_back(r.bbox.x + r.bbox.width/2);
            ys.push_back(r.bbox.y + r.bbox.height/2);
        }
        round_to_group_lb(xs);
        round_to_group_lb(ys);
        vector<tuple<int, int, int>> rank;
        for (int i = 0; i < rooms.size(); ++i) {
            rank.push_back(make_tuple(-ys[i], xs[i], i));
        }
        sort(rank.begin(), rank.end());
        order->clear();
        for (auto t: rank) {
            order->push_back(get<2>(t));
        }
    }

    template <int SIDE>
    int find_popular_value (vector<int> const &a, int min, int max, int th) {
        std::unordered_map<int, int> cnt;
        int cc = 0;
        for (int v: a) {
            int hit = 0;
            for (int i = 0; i < th; ++i) {
                int v2 = v + SIDE * i;
                if (v2 < min) continue;
                if (v2 > max) continue;
                hit += 1;
                cnt[v2] += 1;
            }
            if (hit > 0) ++cc;
        }
        std::pair<int, int> best(-1, -1);
        for (auto p: cnt) {
            if (p.second > best.second) {
                best = p;
            }
        }
        LOG(INFO) << "POPULARITY: " << (1.0 * best.second / cc);
        if (cc == 0) return -1;
        if (cc * POPULAR_VALUE_TH > best.second) return -1;
        return best.first;
    }


    template <int D>
    void check_switch_index (int i, int j, int *pi, int *pj) {
        if (D == 0) {
            *pi = i;
            *pj = j;
        }
        else {
            *pi = j;
            *pj = i;
        }
    }

    template <int D>
    void squeeze (cv::Mat room, vector<int> const &lbs,
                                vector<int> const &ubs,
                                int popular_l, int popular_u, int th) {
        int n = (D == 0) ? room.rows : room.cols;
        CHECK(n == lbs.size() && n == ubs.size());
        int a, b;
        for (int i = 0; i < n; ++i) {
            int lb = lbs[i], ub = ubs[i];
            //   [lb, ub] = 1,   outside are 0
            if ((popular_l >= 0) && (lb > popular_l - th)) {
                for (int j = lb; j < popular_l; ++j) {
                    check_switch_index<D>(i, j, &a, &b);
                    room.at<uint8_t>(a, b) = 0;
                }
            }
            if ((popular_u >= 0) && (ub < popular_u + th)) {
                for (int j = popular_u + 1; j <= ub; ++j) {
                    check_switch_index<D>(i, j, &a, &b);
                    room.at<uint8_t>(a, b) = 0;
                }
            }
        }
    }

    void regularize_room (cv::Mat room, int pixels = 3) {
        CHECK(room.type() == CV_8U);
        vector<int> left(room.rows, room.cols); // x: left -> right
        vector<int> right(room.rows, -1);
        vector<int> top(room.cols, room.rows);  // y: top -> bottom
        vector<int> bottom(room.cols, -1);
        for (int i = 0; i < room.rows; ++i) {
            uint8_t const *row = room.ptr<uint8_t const>(i);
            for (int j = 0; j < room.cols; ++j) {
                if (row[j] == 0) continue;
                if (j < left[i]) left[i] = j;
                if (j > right[i]) right[i] = j;
                if (i < top[j]) top[j] = i;
                if (i > bottom[j]) bottom[j] = i;
            }
        }
        int th = POPULAR_TH;
        int l, u;
        l = find_popular_value<1>(left, 0, room.cols-1, th);
        u = find_popular_value<-1>(right, 0, room.cols-1, th);
        squeeze<0>(room, left, right, l, u, th);
        l = find_popular_value<1>(top, 0, room.rows-1, th);
        u = find_popular_value<-1>(bottom, 0, room.rows-1, th);
        squeeze<1>(room, top, bottom, l, u, th);
    }

    bool test_intersect (Contour const &c1, Object const &obj, int relax) {
        // TODO: 注意：如果c2完全包含c1，则会返回false BUG?
        int x1 = obj.bbox.x - relax;
        int y1 = obj.bbox.y - relax;
        int x2 = obj.bbox.x + obj.bbox.width + relax;
        int y2 = obj.bbox.y + obj.bbox.height + relax;
        if (cv::pointPolygonTest(c1, cv::Point(x1, y1), true) >= 0) return true;
        if (cv::pointPolygonTest(c1, cv::Point(x1, y2), true) >= 0) return true;
        if (cv::pointPolygonTest(c1, cv::Point(x2, y2), true) >= 0) return true;
        if (cv::pointPolygonTest(c1, cv::Point(x2, y1), true) >= 0) return true;
        return false;
    }



	bool box_contains_line (cv::Rect const &rect, Line const &l) {
        // TODO
        if (rect.contains(l[0])) return true;
        if (rect.contains(l[1])) return true;
        return false;
		//The Liang-Barsky algorithm is a cheap way to find the intersection points between a line segment and an axis-aligned rectangle. 
        // https://gist.github.com/ChickenProp/3194723
#if 0
        int dx = l[1].x - l[0].x;
        int dy = l[1].y - l[0].y;

        int p[] = {-dx, dx, -dy, dy};
        int q[] = {l[0].x - rect.x,                 // > 0:  l[0]是否在rect左侧
                   rect.x + rect.width - l[0].x,    // 
                   l[0].y - rect.y,
                   rect.y + rect.height - l[0].y
                    };
        var u1 = Math.NEGATIVE_INFINITY;
        var u2 = Math.POSITIVE_INFINITY;

        for (int i = 0; i < 4; ++i) {
            if (p[i] == 0) {
                if (q[i] < 0) return false;
            }
            else {
                var t = q[i] / p[i];
                if (p[i] < 0 && u1 < t)
                        u1 = t;
                else if (p[i] > 0 && u2 > t)
                        u2 = t;
            }
        }

        if (u1 > u2 || u1 > 1 || u1 < 0) return false;

        return true;
#endif
	}

    void filter_objects_by_contour(Contour const &contour,
                                   ObjectLayers const &from,
                                   ObjectLayers *to) {
        for (int i = 0; i < from.size(); ++i) {
            //to->at(i).clear();
            for (auto const &obj: from[i]) {
                if (test_intersect(contour, obj, OBJECT_ROOM_TEST_RELAX)) {
                    to->at(i).push_back(obj);
                }
            }
        }
    }

    void filter_objects_by_bbox (cv::Rect const &bbox,
                                   ObjectLayers const &from,
                                   ObjectLayers *to) {
        for (int i = 0; i < from.size(); ++i) {
            //to->at(i).clear();
            for (auto const &obj: from[i]) {
                cv::Rect inter = bbox & obj.bbox;
                if (inter.width > 0 && inter.height > 0) {
                    to->at(i).push_back(obj);
                }
            }
        }
    }

    bool align_object_to_room (Contour const &contour, Object const &obj, Object *tmp) {
        vector<method2::QLines> qlines;
        method2::add_qlines(&qlines, obj.bbox, true, ALIGN_TO_ROOM_RELAX);
        vector<cv::Point> pts;
        for (int i = 0; i < contour.size(); ++i) {
            auto const &p1 = contour[i];
            auto const &p2 = contour[(i+1) % contour.size()];
            if (p1 == p2) continue;
            XLine l(p1, p2);
            for (auto const &q: qlines) {
                for (auto const &l2: q) {
                    if (parallel(l, l2)) continue;
                    cv::Point x = method2::intersect(l, l2);
                    if (method2::try_extend(l2, x, true, 0, 0).hit && method2::try_extend(l, x, true, 0, 0).hit) {
                        pts.push_back(x);
                    }
                }
            }
        }
        if (pts.size() < 2) return false;
        tmp->bbox = bound(pts);
        box2contour(tmp->bbox, &tmp->contour);
        return true;
    }

    // TODO: speedup
    void align_objects_to_room (Contour const &contour,
                                   ObjectLayers const &from,
                                   ObjectLayers *to) {
        for (int i = 0; i < from.size(); ++i) {
            //to->at(i).clear();
            for (auto const &obj: from[i]) {
                Object tmp;
                if (align_object_to_room(contour, obj, &tmp)) {
                    to->at(i).push_back(tmp);
                }
            }
        }
    }

    map<string, int> const WDAX_LOOKUP{
        {"WDAS_WALLS", CAD::WDAS_WALLS},
        {"WDAS_PILLARS", CAD::WDAS_PILLARS},
        {"WDAS_DOORS", CAD::WDAS_DOORS},
        {"WDAS_SAFETY_DOORS", CAD::WDAS_SAFETY_DOORS},
        {"WDAS_FIRE_HYDRANT", CAD::WDAS_FIRE_HYDRANT},
        {"WDAS_OBSTACL", CAD::WDAS_OBSTACLE},
        {"WDAS_OBSTACLE", CAD::WDAS_OBSTACLE},
        {"WDAS_GUARD", CAD::WDAS_GUARD},
        {"WDAF_PASSAGE", CAD::WDAF_PASSAGE},
        {"WDAF_DOCK", CAD::WDAF_DOCK},
        {"WDAF_DOCK_IN", CAD::WDAF_DOCK_IN},
        {"WDAF_DOCK_OUT", CAD::WDAF_DOCK_OUT},
        {"WDAF_MINIROOM", CAD::WDAF_MINIROOM}, 
        {"WDAS_MINIROOM", CAD::WDAF_MINIROOM},       // 容错
        {"WDAX_FORKLIFT", CAD::WDAX_FORKLIFT}, 
        {"WDAX_AVG", CAD::WDAX_AVG},
        {"WDAX_MANUP_TRUCK", CAD::WDAX_MANUP_TRUCK},
        {"WDAX_PPERATING_PLATFORM", CAD::WDAX_PPERATING_PLATFORM},
        {"WDAX_WAREHOUSE_OPERATOR", CAD::WDAX_WAREHOUSE_OPERATOR},
        {"WDAX_CONVEY_LINE", CAD::WDAX_CONVEY_LINE}
    };

    map<string, int> const ANNO_LOOKUP{
        {"HINT_ROI", CAD::ANNO_HINT_ROI},
        {"HINT_CLEAR", CAD::ANNO_HINT_CLEAR},
        {"HINT_DOCK", CAD::ANNO_HINT_DOCK},
        {"HINT_DOCK_IN", CAD::ANNO_HINT_DOCK_IN},
        {"HINT_DOCK_OUT", CAD::ANNO_HINT_DOCK_OUT},
        {"HINT_CONNECT", CAD::ANNO_HINT_CONNECT},
        {"HINT_DROP_ROOM", CAD::ANNO_HINT_DROP_ROOM},
        {"GUARD_OBSTACLE", CAD::ANNO_GUARD_OBSTACLE},
        {"GUARD_PASSAGE", CAD::ANNO_GUARD_PASSAGE},
        {"GUARD_MINIROOM", CAD::ANNO_GUARD_MINIROOM},
        {"GUARD_FORKLIFT", CAD::ANNO_GUARD_FORKLIFT},
        {"GUARD_AVG", CAD::ANNO_GUARD_AVG},
        {"GUARD_MANUP_TRUCK", CAD::ANNO_GUARD_MANUP_TRUCK},
        {"GUARD_PPERATING_PLATFORM", CAD::ANNO_GUARD_PPERATING_PLATFORM},
        {"ANNO_GUARD_WAREHOUSE_OPERATOR", CAD::ANNO_GUARD_WAREHOUSE_OPERATOR},
        {"GUARD_CONVEY_LINE", CAD::ANNO_GUARD_CONVEY_LINE}
    };

    vector<int> const DOOR_LIKE_LAYERS{CAD::WDAS_DOORS,
                                       CAD::WDAX_EXITS,
                                       CAD::WDAX_DOCK,
                                       CAD::WDAX_DOCK_IN,
                                       CAD::WDAX_DOCK_OUT,
                                       CAD::WDAS_SAFETY_DOORS};


    map<int, int> const MAP_WDAX_FIXTURE_CODE{
        {CAD::WDAS_WALLS, Fixture::WALL},
        {CAD::WDAS_PILLARS, Fixture::COLUMN},
        {CAD::WDAS_DOORS, Fixture::DOOR},
        {CAD::WDAX_EXITS, Fixture::EXIT},
        {CAD::WDAX_DOCK, Fixture::DOCK},
        {CAD::WDAX_DOCK_IN, Fixture::DOCK_IN},
        {CAD::WDAX_DOCK_OUT, Fixture::DOCK_OUT},
        {CAD::WDAS_SAFETY_DOORS, Fixture::SAFETY_DOOR},
        {CAD::WDAS_FIRE_HYDRANT, Fixture::FIRE_HYDRANT},
        {CAD::WDAS_OBSTACLE, Fixture::OBSTACLE},
        {CAD::WDAS_GUARD, Fixture::GUARD},
        {CAD::WDAF_PASSAGE, Fixture::GUARD_PASSAGE},
        {CAD::WDAF_MINIROOM, Fixture::GUARD},
        {CAD::WDAX_FORKLIFT, Fixture::FORKLIFT},
        {CAD::WDAX_AVG, Fixture::AVG},
        {CAD::WDAX_MANUP_TRUCK, Fixture::MANUP_TRUCK},
        {CAD::WDAX_PPERATING_PLATFORM, Fixture::PPERATING_PLATFORM},
        {CAD::WDAX_WAREHOUSE_OPERATOR, Fixture::WAREHOUSE_OPERATOR},
        {CAD::WDAX_CONVEY_LINE, Fixture::CONVEY_LINE},
        {CAD::WDAX_GUARD_2M, Fixture::GUARD_2M},
        {CAD::WDAX_ACC_GUARD, Fixture::ACC_GUARD},
    };

    py::list create_py_contour (Contour const &c) {
        py::list l;
        for (auto const &p: c) {
            py::list pp;
            pp.append(p.x);
            pp.append(p.y);
            l.append(pp);
        }
        return l;
    }

    py::dict create_py_room (int cc, bool miniroom, ObjectLayers const &layers) {
        py::list fixtures;
        array<int, CAD::NUM_LAYERS> indices;
        for (int i = 0; i < indices.size(); ++i) indices[i] = i;
        // 把guard换到前面以免显示的时候覆盖住别的东西
        std::swap(indices[CAD::WDAS_PILLARS], indices[CAD::WDAS_GUARD]);
        for (int i: indices) {
            auto it = MAP_WDAX_FIXTURE_CODE.find(i);
            if (it == MAP_WDAX_FIXTURE_CODE.end()) continue;
            if (layers[i].empty()) continue;
            int code = it->second;
            //LOG(INFO) << "OBJ " << code << ": " << layers[i].size();
            for (auto const &c: layers[i]) {
                py::dict fixture;
                fixture["polygon"] = create_py_contour(c.contour);
                fixture["type"] = code;
                fixture["height"] = -1;
                fixture["effective"] = true;
                if (code == Fixture::WALL) {
                    fixture["miniroom"] = miniroom;
                }
                fixtures.append(fixture);
            }
        }
        py::dict room;
        room["cc"] = cc;
        room["fixtures"] = fixtures;
        return room;
    }

    void merge_boxes_to_objects (vector<cv::Rect> const &boxes, Objects *objs) {
        for (auto const &box: boxes) {
            Object obj;
            obj.bbox = box;
            box2contour(box, &obj.contour);
            objs->push_back(obj);
        }
    }

    void cleanup_object_layers (ObjectLayers &obj_layers, Contours const &borders) {
        obj_layers[CAD::DEFAULT].clear();
        Objects doors;
        doors.swap(obj_layers[CAD::WDAS_DOORS]);

        vector<pair<Objects const*, Objects*>> dock_tests{
            {&obj_layers[CAD::WDAF_DOCK], &obj_layers[CAD::WDAX_DOCK]},
            {&obj_layers[CAD::WDAF_DOCK_IN], &obj_layers[CAD::WDAX_DOCK_IN]},
            {&obj_layers[CAD::WDAF_DOCK_OUT], &obj_layers[CAD::WDAX_DOCK_OUT]}};

        // 把DOORS分类成DOORS, EXITS, DOCK{_IN/OUT}
        for (auto &door: doors) {
            int l = std::max(door.bbox.width, door.bbox.height);
            if (l < DOOR_EXIT_THRESHOLD) {
                obj_layers[CAD::WDAX_EXITS].push_back(std::move(door));
                continue;
            }
            // 测试各种dock, dock_in, dock_out
            bool used = false;
            // TODO: 如果WDAF_DOCK*有重叠，可能会导致一个dock被分入多个目标图层
            for (auto p: dock_tests) {
                Objects const *tests = p.first;
                Objects *targets = p.second;
                for (Object const &bb: *tests) {
                    if (test_intersect(bb.contour, door, 0)) {
                        targets->push_back(door);
                        used = true;
                        break;
                    }
                }
            }
            if (!used) {
                // 测试是否是dock
                Contour pts;
                {
                    int const relax = OBJECT_ROOM_TEST_RELAX;
                    cv::Rect bb = door.bbox;
                    bb.x -= relax;
                    bb.y -= relax;
                    bb.width += 2 * relax;
                    bb.height += 2 * relax;
                    box2contour(bb, &pts);
                }
                int all_in = true; // 所有的点全在墙内
                for (auto const &pt: pts) {
                    int ii = 0;
                    while (ii < borders.size()) {
                        auto const &c = borders[ii];
                        if (cv::pointPolygonTest(c, pt, true) > 0) break;
                        if (cv::pointPolygonTest(c, pt, true) > 0) break;
                        if (cv::pointPolygonTest(c, pt, true) > 0) break;
                        if (cv::pointPolygonTest(c, pt, true) > 0) break;
                        ++ii;
                    }
                    if (ii >= borders.size()) { // 该点不在任何墙内
                        all_in = false;
                        break;
                    }
                }
                if (!all_in) { // 有点在墙外就是dock
                    used = true;
                    obj_layers[CAD::WDAX_DOCK].push_back(std::move(door));
                }
            }
            if (!used) {
                obj_layers[CAD::WDAS_DOORS].push_back(std::move(door));
            }
        }

        // 处理消防栓
        // http://gitlab.shucheng-ai.com:38080/wda/wda-doc/issues/34
        for (auto const &fire: obj_layers[CAD::WDAS_FIRE_HYDRANT]) {
            // 找最近的柱子以及距离
            cv::Point ct = fire.center();
            double min_dist = 0;
            Object const *nearest = nullptr;
            for (auto const &pillar: obj_layers[CAD::WDAS_PILLARS]) {
                double dist = cv::norm(ct - pillar.center());
                if (nearest == nullptr || dist < min_dist) {
                    min_dist = dist;
                    nearest = &pillar;
                }
            }
            if (nearest) { // 确定是否足够近
                cv::Rect inter = fire.bbox & relax_box(nearest->bbox, FIRE_HYDRANT_PILLAR_GAP);
                // 太远 废掉
                if (inter.width <= 0) nearest = nullptr;
            }
            if (nearest) {
                cv::Point dir = ct - nearest->center();
                int dw = (fire.bbox.width + nearest->bbox.width)/2;
                int dh = (fire.bbox.height + nearest->bbox.height)/2;
                // 测试在左右还是在上下
                Object guard;
                if (std::abs(std::abs(dir.x) - dw) < std::abs(std::abs(dir.y) - dh)) {
                    if (dir.x >= 0) {
                        // fire on right of pillar
                        guard.bbox = cv::Rect(ct.x, ct.y - FIRE_ACC_GUARD_SIZE/2,
                                         FIRE_ACC_GUARD_SIZE, FIRE_ACC_GUARD_SIZE);
                    }
                    else {
                        // fire on left of pillar
                        guard.bbox = cv::Rect(ct.x+fire.bbox.width-FIRE_ACC_GUARD_SIZE,
                                         ct.y - FIRE_ACC_GUARD_SIZE/2,
                                         FIRE_ACC_GUARD_SIZE, FIRE_ACC_GUARD_SIZE);
                    }

                }
                else {
                    if (dir.y > 0) {
                        // fire on top of pillar
                        guard.bbox = cv::Rect(ct.x-FIRE_ACC_GUARD_SIZE/2, ct.y,
                                         FIRE_ACC_GUARD_SIZE, FIRE_ACC_GUARD_SIZE);
                    }
                    else {
                        // fire on bottom of pillar
                        guard.bbox = cv::Rect(ct.x-FIRE_ACC_GUARD_SIZE/2,
                                         ct.y+fire.bbox.height-FIRE_ACC_GUARD_SIZE,
                                         FIRE_ACC_GUARD_SIZE, FIRE_ACC_GUARD_SIZE);
                    }
                }
                box2contour(guard.bbox, &guard.contour);
                obj_layers[CAD::WDAX_ACC_GUARD].push_back(guard);
            }
            else {  // 假设在墙上
                Object guard;
                guard.bbox = relax_box(ct, FIRE_GUARD_2M_SIZE);
                box2contour(guard.bbox, &guard.contour);
                obj_layers[CAD::WDAX_GUARD_2M].push_back(guard);
            }
        }

        // miniroom全都变成GUARD
        for (auto &miniroom: obj_layers[CAD::WDAF_DOCK]) {
            obj_layers[CAD::WDAS_OBSTACLE].push_back(miniroom);
        }

        obj_layers[CAD::WDAF_DOCK].clear();
        obj_layers[CAD::WDAF_DOCK].clear();
        obj_layers[CAD::WDAF_DOCK_IN].clear();
        obj_layers[CAD::WDAF_DOCK_OUT].clear();
    }

    void add_guards_one_segment (cv::Point p1, cv::Point p2, int dist, Objects *guards) {
        cv::Point d = p2 - p1;
        cv::Point2f d1(-d.y, d.x);    // 90 degree counter
        {
            float l = cv::norm(d1);
            if (l == 0) return;
            d1 *= dist / l;
        }
        cv::Point2f s1 = p1;
        cv::Point2f s2 = p2;
        cv::Point2f s3 = s2 + d1;
        cv::Point2f s4 = s1 + d1;
        guards->emplace_back();
        Object &obj = guards->back();
        obj.contour.emplace_back(s1);
        obj.contour.emplace_back(s2);
        obj.contour.emplace_back(s3);
        obj.contour.emplace_back(s4);
        obj.bbox = bound(obj.contour);
    }

    void add_wall_guards (Contour const &contour, int dist, Objects *guards) {
        // wall contour is counter-clockwise
        for (int i = 0; i < contour.size(); ++i) {
            cv::Point const &p1 = contour[i];
            cv::Point const &p2 = contour[(i+1) % contour.size()];
            add_guards_one_segment(p1, p2, dist, guards);
        }
    }

    struct RoomInfo {
        int cc = -1;
        Object walls;
        bool miniroom = false;
        cv::Mat thumbnail;
    };

    class CadProcessor {
        CAD cad;
        int current_layer;
    public:
        CadProcessor (): current_layer(CAD::DEFAULT) {
        }

        // select layer to inject
        bool select (string const &layer) {
            auto it = WDAX_LOOKUP.find(layer);
            if (it == WDAX_LOOKUP.end()) return false;
            current_layer = it->second;
            CHECK(current_layer >= 0);
            CHECK(current_layer < cad.layers.size());
            return true;
        }

        void add (float x1f, float y1f, float x2f, float y2f) {
            int x1 = int(round(x1f));
            int x2 = int(round(x2f));
            int y1 = int(round(y1f));
            int y2 = int(round(y2f));
            cad.layers[current_layer].lines.emplace_back(Line{cv::Point(x1, y1), cv::Point(x2, y2)});
        }

        void annotate (string const &annotation, int x1, int y1, int x2, int y2) {
            auto it = ANNO_LOOKUP.find(annotation);
            if (it == ANNO_LOOKUP.end()) {
                LOG(INFO) << "ANNOTATION NOT RECOGNIZED: " << annotation;
                return;
            }
            cad.annotations[it->second].push_back(cv::Rect(cv::Point(x1, y1), cv::Point(x2, y2)));
        }

        py::list extract (string const &root) {

            vector<cv::Rect> ccs;
            extract_cc(cad, &ccs, CC_RELAX);
            LOG(INFO) << "CCS: " << ccs.size();

            ObjectLayers all_objs;
            //ObjectLayers all_objs_test;

            for (int i = 0; i < CAD::NUM_LAYERS; ++i) {
                if (i == CAD::WDAS_WALLS) continue;
                Contours contours;
                /*
                */
                vector<cv::Rect> bbs;
                extract_cc(cad, &bbs, OBJECT_RELAX, i);

                Objects &objs = all_objs[i];
                //Contours &objs_test = all_objs_test[i];
                for (auto &bb: bbs) { // boxes to contours
                    objs.emplace_back();
                    objs.back().bbox = bb;
                    box2contour(bb, &objs.back().contour);
                    /*
                    bb.x -= OBJECT_ROOM_TEST_RELAX;
                    bb.y -= OBJECT_ROOM_TEST_RELAX;
                    bb.width += 2 * OBJECT_ROOM_TEST_RELAX;
                    bb.height += 2 * OBJECT_ROOM_TEST_RELAX;
                    objs_test.emplace_back();
                    box2contour(bb, &objs_test.back());
                    */
                }
            }
            // move annotations to objects
            merge_boxes_to_objects(cad.annotations[CAD::ANNO_HINT_DOCK], &all_objs[CAD::WDAF_DOCK]);
            merge_boxes_to_objects(cad.annotations[CAD::ANNO_HINT_DOCK_IN], &all_objs[CAD::WDAF_DOCK_IN]);
            merge_boxes_to_objects(cad.annotations[CAD::ANNO_HINT_DOCK_OUT], &all_objs[CAD::WDAF_DOCK_OUT]);
            merge_boxes_to_objects(cad.annotations[CAD::ANNO_GUARD_OBSTACLE], &all_objs[CAD::WDAS_GUARD]);
            merge_boxes_to_objects(cad.annotations[CAD::ANNO_GUARD_PASSAGE], &all_objs[CAD::WDAF_PASSAGE]);
            merge_boxes_to_objects(cad.annotations[CAD::ANNO_GUARD_MINIROOM], &all_objs[CAD::WDAF_MINIROOM]);
            merge_boxes_to_objects(cad.annotations[CAD::ANNO_GUARD_FORKLIFT], &all_objs[CAD::WDAX_FORKLIFT]);
            merge_boxes_to_objects(cad.annotations[CAD::ANNO_GUARD_AVG], &all_objs[CAD::WDAX_AVG]);
            merge_boxes_to_objects(cad.annotations[CAD::ANNO_GUARD_MANUP_TRUCK], &all_objs[CAD::WDAX_MANUP_TRUCK]);
            merge_boxes_to_objects(cad.annotations[CAD::ANNO_GUARD_PPERATING_PLATFORM], &all_objs[CAD::WDAX_PPERATING_PLATFORM]);
            merge_boxes_to_objects(cad.annotations[CAD::ANNO_GUARD_WAREHOUSE_OPERATOR], &all_objs[CAD::WDAX_WAREHOUSE_OPERATOR]);
            merge_boxes_to_objects(cad.annotations[CAD::ANNO_GUARD_CONVEY_LINE], &all_objs[CAD::WDAX_CONVEY_LINE]);

            // 被WDAF_PASSAGE覆盖的pillars都需要干掉
            {
                Objects pillars;
                pillars.swap(all_objs[CAD::WDAS_PILLARS]);
                auto const &passages = all_objs[CAD::ANNO_GUARD_PASSAGE];
                for (auto &pillar: pillars) {
                // TODO: 进行更完整的测试和切分
                // 如果有一半在通道里，应该把不在的部分切出来。
                    bool hit = false;
                    for (auto const &pt: pillar.contour) {
                        for (auto const &passage: passages) {
                            if (passage.bbox.contains(pt)) {
                                hit = true;
                                goto found_hit;
                            }
                        }
                    }
found_hit:          if (!hit) {
                        all_objs[CAD::WDAS_PILLARS].push_back(pillar);
                    }
                }
            }



            vector<RoomInfo> rooms;
            Contours borders;   // 所有的仓间连续部分的外围多边形
                                    // 用于判断仓间门 vs dock
            int cc_id = 0;
            py::list pyccs;

            for (auto const &cc: ccs) {
                std::pair<int,int> minmax = std::minmax(cc.width, cc.height);
                if (minmax.first < ROOM_SIZE_MIN_THRESHOLD) continue;
                if (minmax.second < ROOM_SIZE_MAX_THRESHOLD) continue;


                {
                    RasterCanvas vis(cc, 2048, 20, CV_8U);
                    vis.draw(cad, cv::Scalar(255));
                    ilog.log(vis.image, format("ccv_%d.png") % cc_id);
                }

                Objects walls;
                Objects internal_holes;
                Contours bds;
                extract_rooms(cad, all_objs, cc, cc_id, &walls, &bds, &internal_holes);

                LOG(INFO) << "GOT OBSTACLES " << internal_holes.size();
                all_objs[CAD::WDAS_OBSTACLE].insert(all_objs[CAD::WDAS_OBSTACLE].end(),
                                internal_holes.begin(), internal_holes.end());

                if (walls.empty()) continue;

                // bds合并入borders
                for (auto &c: bds) {
                    borders.emplace_back();
                    borders.back().swap(c);
                }

        		RasterCanvas thumbnail(cc, 256, 10, CV_8UC3);
                thumbnail.draw(cad, cv::Scalar(255, 255, 255));

                vector<int> order;
                sort_rooms(walls, &order);

                // 把minirooms加入
                int normal_walls = walls.size();
                for (auto const &miniroom: all_objs[CAD::WDAF_MINIROOM]) {
                    for (int xxx = 0; xxx < normal_walls; ++xxx) {
                        cv::Rect sect = miniroom.bbox & walls[xxx].bbox;
                        if (sect.width > 0 && sect.height > 0) {
                            // 碰撞了，加入miniroom
                            Object intersect;
                            contour_intersect(miniroom, walls[xxx], &intersect);
                            if (intersect.contour.size()) {
                                order.push_back(walls.size());
                                walls.push_back(intersect);
                                break;
                            }
                        }
                    }
                }

                for (int room_index: order) {
                    rooms.emplace_back();
                    RoomInfo &room = rooms.back();
                    room.cc = cc_id;

                    room.walls = walls[room_index];
                    room.miniroom = room_index >= normal_walls;

                    room.thumbnail = thumbnail.image.clone();
                    thumbnail.draw(room.walls.bbox, cv::Scalar(0, 255, 0), 2);
                    cv::swap(thumbnail.image, room.thumbnail);
                }
                {
                    py::list l2;
                    l2.append(cc.x);
                    l2.append(cc.y);
                    l2.append(cc.x + cc.width);
                    l2.append(cc.y + cc.height);
                    pyccs.append(l2);
                }
                ++cc_id;
            }

            cleanup_object_layers(all_objs, borders); //, rooms);

            ObjectLayers all_door_like;
            for (int l: DOOR_LIKE_LAYERS) {
                all_door_like[l].swap(all_objs[l]);
            }

            py::list pyrooms;
            for (int room_id = 0; room_id < rooms.size(); ++room_id) {
                auto &room = rooms[room_id];
                if (root.size()) {
                    system((format{"mkdir -p \"%1%/%2%\""} % root % room_id).str().c_str());
                    cv::imwrite((format("%1%/%2%/thumbnail.png") % root % room_id).str(), room.thumbnail);
                }
                ObjectLayers layers;
                //filter_objects_by_contour(room.walls.contour, all_objs, &layers);
                filter_objects_by_bbox(room.walls.bbox, all_objs, &layers);
                if (room.miniroom) { 
                    // 如果仓间本身是miniroom，判断碰撞时会和作为障碍物的自己碰撞
                    // 所以如果仓间本身是miniroom, 则minirooms不再作为障碍物加入
                    layers[CAD::WDAF_MINIROOM].clear();
                }
                align_objects_to_room(room.walls.contour, all_door_like, &layers);
                int wall_guard_dist = room.miniroom ? 0 : WALL_GUARDS_DIST;
                add_wall_guards(room.walls.contour, wall_guard_dist, &layers[CAD::WDAS_GUARD]);
                layers[CAD::WDAS_WALLS].push_back(room.walls);
                pyrooms.append(create_py_room(room.cc, room.miniroom, layers));
            }
            return py::make_tuple(pyrooms, pyccs);
        }

    void extract_cc_rooms (vector<Line> &lines, Box bb, Detection *det) {
                           
            
            Layer connected;
            // extract rooms
            //RasterCanvas cvs(cc_bb, 2048, 20);
            RasterCanvas cvs(cc_bb, 8192, 20);
            //connect_lines(cad.layers[CAD::WDAS_WALLS].lines, layers, &connected.lines);
            //connected.lines = cad.layers[CAD::WDAS_WALLS].lines;
            cvs.draw(connected, cv::Scalar(1));

            if (ilog.enabled()) {
                RasterCanvas fuck(cc_bb, 1024, 20, CV_8UC3);
                fuck.draw(connected, cv::Scalar(0, 255, 0));
                ilog.log(fuck.image, format("walls_%sc.png") % cc_id);
                fuck.draw(cad.layers[CAD::WDAS_WALLS], cv::Scalar(0, 0, 255));
                fuck.draw(cad.layers[CAD::WDAS_DOORS], cv::Scalar(255, 0, 0));
                fuck.draw(cad.layers[CAD::WDAS_SAFETY_DOORS], cv::Scalar(255, 0, 0));
                ilog.log(fuck.image, format("walls_%sd.png") % cc_id);
            }

            /*
            if (cad.annotations[CAD::ANNO_HINT_ROI].size()) {
                // apply annotation
                cv::Mat mask(cvs.image.rows, cvs.image.cols, CV_8U, cv::Scalar(0));
                for (cv::Rect roi: cad.annotations[CAD::ANNO_HINT_ROI]) {
                    cvs.map(&roi);
                    cv::rectangle(mask, roi, cv::Scalar(1), -1);
                }
                cvs.image &= mask;
            }
            */

            for (cv::Rect roi: cad.annotations[CAD::ANNO_HINT_CLEAR]) {
                cvs.map(&roi);
                cv::rectangle(cvs.image, roi, cv::Scalar(0), -1);
            }

            // 外边界填充2
            cv::floodFill(cvs.image, cv::Point(0,0), cv::Scalar(2));
            // cvs: 外侧2, 边界和物体1, 空的地方0
            {
                cv::Mat border;
                cv::threshold(cvs.image, border, 1, 1, cv::THRESH_BINARY_INV);
                // border: 外侧0, 其余1
                ilog.log(border * 255, format("border_%s.png") % cc_id);
                borders->clear();
                cv::findContours(border, *borders, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_TC89_KCOS);
                // findContours modifies border, so discard border here by scope
            }

            // cvs: 外侧1, 边界和物体1, 空的地方0
            cv::threshold(cvs.image, cvs.image, 1, 1, cv::THRESH_TRUNC);
            ilog.log(cvs.image * 255, format("trunc_%s.png") % cc_id);

            cv::Mat labels, stats, centroids;
            // holes: 外侧0, 边界和物体0, 空的地方1
            cv::Mat holes = cv::Scalar::all(1) - cvs.image;
            ilog.log(holes * 255, format{"cc_%d.png"} % cc_id);
            int n = cv::connectedComponentsWithStats(holes, labels, stats, centroids, 4, CV_32S);
            vector<Object> rooms;
            cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(OPEN_KERNEL, OPEN_KERNEL));

            obstacles->clear();


            for (int i = 1; i < n; ++i) {
                std::pair<int,int> minmax = std::minmax(cvs.unscale(stats.at<int>(i, cv::CC_STAT_WIDTH)),
                                            cvs.unscale(stats.at<int>(i, cv::CC_STAT_HEIGHT)));
                if (minmax.first < ROOM_SIZE_MIN_THRESHOLD) continue;
                if (minmax.second < ROOM_SIZE_MAX_THRESHOLD) continue;
                // refine room contour

                cv::Mat room = (labels == i);   // 0 or 255
                CHECK(room.type() == CV_8U); 
                cv::threshold(room, room, 1, 1, cv::THRESH_BINARY); // -> 0/1

                {
                    bool drop = false;
                    for (cv::Rect roi: cad.annotations[CAD::ANNO_HINT_DROP_ROOM]) {
                        roi &= cc_bb;
                        if (roi.width <= 0) continue;
                        if (roi.height <= 0) continue;
                        cvs.map(&roi);
                        cv::Mat sub(room, roi);
                        double min, max;
                        cv::minMaxLoc(sub, &min, &max);
                        if (max > 0) {
                            drop = true;
                            break;
                        }
                    }
                    // 放弃仓间
                    if (drop) continue;
                }


                
                // cv::drawContours(room, contours, i, cv::Scalar(1), -1);
                cv::morphologyEx(room, room, cv::MORPH_CLOSE, kernel);
                // CLOSE可能会close出窟窿来，需要填上
                // room: 房间内可用的地方是1, 其余地方是0
                cv::Mat closed_room = room.clone();
                cv::floodFill(closed_room, cv::Point(0,0), cv::Scalar(2));
                // closed_room: 外侧是2, 可用的地方是1, 内部的洞是0
                cv::threshold(closed_room, closed_room, 1, 1, cv::THRESH_BINARY_INV);
                // closed_room: 外侧是0, 内侧是1
                regularize_room(closed_room);
                Contours contours1;
                cv::findContours(closed_room.clone(), contours1, CV_RETR_LIST, CV_CHAIN_APPROX_TC89_KCOS);
                CHECK(contours1.size() == 1);


                rooms.emplace_back();
                rooms.back().contour.swap(contours1[0]);
                // 找出来内部的窟窿, 作为obstacle
                closed_room -= room;
                // closed_room: 内侧不可用的地方是1, 其余是0 
                cv::floodFill(closed_room, cv::Point(0,0), cv::Scalar(2));
                cv::threshold(closed_room, closed_room, 1, 1, cv::THRESH_BINARY_INV);
                // closed_room: 内侧不可用的地方是1, 其余是0 
                cv::findContours(closed_room, contours1, CV_RETR_LIST, CV_CHAIN_APPROX_TC89_KCOS);
                for (auto &c: contours1) {
                    obstacles->emplace_back();
                    obstacles->back().contour.swap(c);
                }
            }

            // unmap to world coordinates
            for (auto &room: rooms) {
                for (auto &p: room.contour) {
                    cvs.unmap(&p);
                }
                room.bbox = bound(room.contour);
            }
            for (auto &obs: *obstacles) {
                for (auto &p: obs.contour) {
                    cvs.unmap(&p);
                }
                obs.bbox = bound(obs.contour);
            }
            for (auto &c: *borders) {
                for (auto &p: c) {
                    cvs.unmap(&p);
                }
            }
            results->swap(rooms);
        }

/// GOOD 

    // 两条平行线段
    class QLines: public array<XLine, 2>  {
    public:
        QLines (Point const &p1, Point const &p2,
                Point const &p3, Point const &p4,
                bool real) {
            at(0) = XLine(p1, p2, real);
            at(1) = XLine(p3, p4, real);
        }

        bool extend (XLine *line, int th) const {
            bool cross = false;
            vector<Extend> es;
            for (auto const &a: *this) {
                if (parallel(*line, a)) continue;
                cv::Point2d c = intersect(*line, a);
                if (try_extend(a, c, true, 0).hit) {  // 必须和框框相交
                    Extend e = try_extend(*line, c, a.real, th);
                    es.push_back(e);
                    cross = cross || e.hit;
                }
            }
            bool u = false;
            if (cross) {
                for (auto const &e: es) {
                    u |= e.apply(line);
                }
            }
            return u;
        }
    };

    class Aux: vector<QLines> {
    public:
        void add (Box const &r, bool real, int relax) {
            CHECK(box[1][0] >> box[0][0]);
            CHECK(box[1][1] >> box[0][1]);
            Point p1 = box[0];  // x0, y0
            Point p2 = box[0];  // x0, y0 -> x1, y0
            Point p3 = box[1];  // x1, y1
            Point p4 = box[1];  // x1, y1 -> x0, y1
            std::swap(p2[0], p4[0]);
            Point dx{relax, 0};
            Point dy{0, relax};
            //  p4 p3
            //  p1 p2
            emplace_back(p1-dx, p2+dx, p4-dx, p3+dx, real);
            emplace_back(p1-dy, p4+dy, p2-dy, p3+dy, real);
        }
    }

    void connect_walls (vector<Line> const &input,
                        vector<Box> const &hint_connect,
                        vector<Line> *output) {

        class Aux aux;
        vector<XLine> lines;
        for (auto const &r: hint_connect) {
            aux.add(hint_connect, false, 0);
        }
        for (auto const &l: lines_in) {
            if (norm(l[0],l[1]) < 100) {
                aux.add(Box(l), true, CONNECT_TH_TINY);
            }
            else {
                lines.emplace_back(l);
            }
        }

        for (auto &line: lines) {
            line.update1 = true;
            line.update2 = false;
        }

        // update1: 当前这次extend是否要考虑延长这根线
        // update2: 新的update1的值，下一轮用; 如果已经测试了无法延长，update2就要变成false
        //          只有延长了的下一次还需要接着测试

        for (;;) {
            int s0 = lines.size();
            for (auto &line: lines) {
                if (!line.update1) continue;
                for (auto const &aux: auxes) {
                    //CHECK(aux.size() == 4);
                    if (aux.extend(&line, CONNECT_TH_MEDIUM)) {
                        line.update2 = true;
                    }
                }
            }

            // 先链接strictly parallel的
            int strict_parallel_connected = 0;

            // 如果有严格平行的被连了，则不要再做更加宽松的测试，
            if (strict_parallel_connected == 0) {
                int i = 0;
                while (i < lines.size()) {
                    int j = i + 1;
                    while (j < lines.size()) {
                        if (!(lines[i].update1 || lines[j].update1)) {
                            ++j;
                            continue;
                        }
                        if (parallel(lines[i], lines[j])) {
                            // merge
                            XLine c;
                            if (connect_parallel(lines[i], lines[j], CONNECT_TH_MEDIUM, &c)) {
                                CHECK(c.norm() >= lines[i].norm());
                                CHECK(c.norm() >= lines[j].norm());
                                lines[i] = c;
                                lines[i].update1 = lines[i].update2 = true;
                                std::swap(lines[j], lines.back());
                                lines.pop_back();
                                continue;
                            }
                        }
                        else if (lines[i].norm() >= MIN_DUAL_EXTEND
                                && lines[j].norm() >= MIN_DUAL_EXTEND) {
                            cv::Point2d c = intersect(lines[i], lines[j]);
                            Extend e1 = try_extend(lines[i], c, false, CONNECT_TH_SMALL);
                            Extend e2 = try_extend(lines[j], c, false, CONNECT_TH_SMALL);
                            if (e1.hit && e2.hit) {
                                if (e1.apply(&lines[i])) {
                                    lines[i].update2 = true;
                                }
                                if (e2.apply(&lines[j])) {
                                    lines[j].update2 = true;
                                }
                            }
                        }
                        ++j;
                    }
                    ++i;
                }
            }
            LOG(INFO) << "REDUCE " << s0 << " => " << lines.size();
            int update = 0;
            for (auto &line: lines) {
                if (line.update2) ++update;
                line.update1 = line.update2;
                line.update2 = false;
            }
            if (update == 0) break;
        }
    }

    class wda_rooms: public Detector {
        double cc_relax;
    public:
        wda_rooms (py::dict conf)
            : cc_relax(conf["cc_relax"].cast<double>()) {

        }

        virtual void detect (View const &view, Detection *det) const {
            vector<Box> ccs;
            vector<Line> lines;
            view.collect(&lines);
            {
                ConnectComponent cc(cc_relax);
                for (auto const &l: lines) {
                    cc.add(l);
                }
                cc.cleanup();
                cc.swap(&ccs);
            }
            vector<Box> hint_connect;
            view.collect_markup_boxes(&hint_connect, Markup::CODE_CONNECT);
        /*
        for (int l: vector<int>{CAD::WDAS_DOORS,
                                CAD::WDAS_SAFETY_DOORS} ) {
            for (auto const &obj: layers[l]) {
                //auxes.emplace_back(relax_box(obj.bbox, CONNECT_TH_SMALL));
                add_qlines(&auxes, obj.bbox, true, CONNECT_TH_TINY);
            }
        }
        convert these to connect hint
        */
            vector<Line> connected;
            connect_walls(lines, hint_connect, &connected);

            //connect_walls(cc_bb, cad.layers[CAD::WDAS_WALLS].lines, layers, cad.annotations[CAD::ANNO_HINT_CONNECT], &connected.lines);
        }
    };

    REGISTER(wda_rooms);

}}
