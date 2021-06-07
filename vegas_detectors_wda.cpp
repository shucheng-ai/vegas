#include "vegas.h"

namespace vegas { namespace detectors {

    static const int CC_RELAX = 5000;
    static const int OBJECT_RELAX = 10; // 1cm
    static const int OBJECT_ROOM_TEST_RELAX = 500;
    static const int ROOM_SIZE_MAX_THRESHOLD = 20000;
    static const int ROOM_SIZE_MIN_THRESHOLD = 15000;
    static const int CONNECT_TH = 8000; // 2米
    static const int CONNECT_TH_TINY = 500; // 2米
    static const int CONNECT_TH_SMALL = 2000; // 2米
    static const int CONNECT_TH_MEDIUM = 8000; // 8米   sample-5xy
    static const int CONNECT_TH_LONG = 10000; // 8米
    static const int MERGE_GAP_TH = 50;
    static const float PARALLEL_TH = 0.5;
    static const int OPEN_KERNEL = 5;
    static const int RANK_GROUP_TH = 10000;
    static const int DOOR_EXIT_THRESHOLD = 2800;
    static const int EXTEND_LINE_THRESHOLD = 1000;
    static const int EXTEND_LINE = 2000;
    static const int FIRE_HYDRANT_PILLAR_GAP = 500;
    static const int FIRE_ACC_GUARD_SIZE = 1500;
    static const int FIRE_GUARD_2M_SIZE = 1000;
    static const int ATTRACT_ASPECT_RATIO = 20;
    static const int ATTRACT_TOLERATE = 5;
    static const int MAX_NON_REAL_EXTEND = 1;
    static const int MIN_DUAL_EXTEND = 2000;
    static const float POPULAR_VALUE_TH = 0.3;
    static const int POPULAR_TH = 3;
    static const int ALIGN_TO_ROOM_RELAX = 1500;
    static const int WALL_GUARDS_DIST = 500;
    static const int WALL_GUARDS_TRIANGULATE = 200;

    // 故意用整数端点
    // 两个端点必须不一样, 构建对象前需要验证
    // 0度，90度是严格计算的(通过判断x或者y是否相等, 不会有舍入误差
    // 别的情况不一定
    // 如果直线是水平，则必须是0度，不会是180度
    // angle in [0, 180)

    struct XLine: Line {
        Line core;
        bool real;    // 点是在实际结构上的
        float length;
        float angle;  // digree, so we can do exact match
                      // 0 - 179
        bool update1 = false;
        bool update2 = false;
        XLine () {}

        XLine (cv::Point p1, cv::Point p2, bool r = true): real(r) {
            cv::Point d = p2 - p1;          // angle: [-pi : pi]
            if (d.y == 0) {
                CHECK(p2.x != p1.x);        // p1 and p2 cannot be equal
                if (p1.x > p2.x) {          // p1 -> p2
                                            // p2.x is bigger
                    std::swap(p1, p2);
                }
                angle = 0;
            }
            else {
                if (d.y < 0) {  
                    std::swap(p1, p2);
                    d = p2 - p1;
                }                               // angle: [0: pi]
                // p2 on top of p1
                if (d.x == 0) {
                    angle = 90;
                }
                else {
                    double r = std::atan2(d.y, d.x);
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
            length = vegas::norm(*this);
        }

        // 长度
        double norm () const {
            return length;
        }
    };

    bool parallel (XLine const &l1, XLine const &l2) {
        float a = l1.angle;
        float b = l2.angle;
        if (a > b) std::swap(a, b);
        // a <= b
        float d = std::min(b - a, a+180-b);
        return d <= PARALLEL_TH;
	}

    namespace method2 { // preferrable

        /*
        void collect_aux_lines (ObjectLayers const &layers, vector<XLine> *aux) {
            aux->clear();
            for (int l: vector<int>{CAD::WDAS_DOORS,
                                    CAD::WDAS_SAFETY_DOORS} ) {
                for (auto const &obj: layers[l]) {
                    append_qlines(aux, l, obj.bbox, CONNECT_TH1);
                }
            }
        }

        double determant (Line const &l1, Line const &l2) {
            Point2d a(l1[1] - l1[0]);
            Point2d b(l2[1] - l2[0]);
            return a.x * b.y - a.y * b.x;
        }

        bool parallel (Line const &l1, Line const &l2) {

        }
        */


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


        // 用四条线段表出的矩形，不能旋转
        class QLines: public array<XLine, 2>  {
        public:
            QLines (cv::Point const &p1, cv::Point const &p2,
                    cv::Point const &p3, cv::Point const &p4,
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

        void add_qlines (vector<QLines> *qlines, cv::Rect const &r, bool real, int relax) {
                int x1 = r.x;
                int y1 = r.y;
                int x2 = r.x + r.width;
                int y2 = r.y + r.height;    //      p4  p3
                cv::Point p1(x1, y1);       //      
                cv::Point p2(x2, y1);       //      p1  p2
                cv::Point p3(x2, y2);
                cv::Point p4(x1, y2);
                cv::Point dx(relax, 0);
                cv::Point dy(0, relax);
                qlines->emplace_back(p1-dx, p2+dx, p4-dx, p3+dx, real);
                qlines->emplace_back(p1-dy, p4+dy, p2-dy, p3+dy, real);
        }


        void connect_walls (cv::Rect bb, vector<Line> const &lines_in,
                            ObjectLayers const &layers, vector<cv::Rect> const &hint_connect,
                            vector<Line> *out) {

            vector<XLine> lines;
            vector<QLines> auxes; 
            for (auto const &r: hint_connect) {
                add_qlines(&auxes, r, false, 0);
            }
            for (auto const &l: lines_in) {
                if (norm(l[0],l[1]) < 100) {
                    add_qlines(&auxes, relax_box(l[0], l[1], 0), true, CONNECT_TH_TINY);
                }
                else {
                    lines.emplace_back(l);
                }
            }
            for (int l: vector<int>{CAD::WDAS_DOORS,
                                    CAD::WDAS_SAFETY_DOORS} ) {
                for (auto const &obj: layers[l]) {
                    //auxes.emplace_back(relax_box(obj.bbox, CONNECT_TH_SMALL));
                    add_qlines(&auxes, obj.bbox, true, CONNECT_TH_TINY);
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
#if 0           // 以下代码是正确的，但是bug通过别的途径解决了，不需要这段检查
                int i = 0;
                while (i < lines.size()) {
                    int j = i + 1;
                    while (j < lines.size()) {
                        if (!(lines[i].update1 || lines[j].update1)) {
                            ++j;
                            continue;
                        }
                        // merge
                        XLine c;
                        if (connect_strict_parallel(lines[i], lines[j], CONNECT_TH_MEDIUM, &c)) {
                            CHECK(c.norm() >= lines[i].norm());
                            CHECK(c.norm() >= lines[j].norm());
                            lines[i] = c;
                            lines[i].update1 = lines[i].update2 = true;
                            std::swap(lines[j], lines.back());
                            lines.pop_back();
                            ++strict_parallel_connected;
                            continue;
                        }
                        ++j;
                    }
                    ++i;
                }
#endif

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
            out->clear();
            for (auto const &l: lines) {
                out->push_back(l);
            }
        }
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


    void extract_rooms (CAD const &cad, ObjectLayers const &layers, cv::Rect cc_bb, int cc_id,
                        vector<Object> *results,
                        vector<Contour> *borders,
                        vector<Object> *obstacles) {
        Layer connected;
        // extract rooms
        RasterCanvas cvs(cc_bb, 2048, 20);
        //connect_lines(cad.layers[CAD::WDAS_WALLS].lines, layers, &connected.lines);
        //connected.lines = cad.layers[CAD::WDAS_WALLS].lines;
        method2::connect_walls(cc_bb, cad.layers[CAD::WDAS_WALLS].lines, layers, cad.annotations[CAD::ANNO_HINT_CONNECT], &connected.lines);
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


    class wda_rooms: public Detector {
    public:
        wda_rooms (py::dict conf) {
        }

        virtual void detect (Layer const &layer, vector<Object> *) const {
        }
    };

    REGISTER(wda_rooms);

}}
