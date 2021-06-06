#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#include "vegas.h"

namespace vegas { namespace extractors {

    namespace ba = boost::accumulators;

    typedef ba::accumulator_set<float, ba::stats<ba::tag::count, ba::tag::mean, ba::tag::variance>> Accumulator;

    /*
    class text_patterns: public Extractor {
    public:
        text_patterns (py::dict conf) {
            LOG(INFO) << "Creating text_patterns extractor";
        }

        virtual size_t dim () const {
        }

        virtual void apply (Layer const &l, double *ft) const {
        }
    };
    */

    class shape_dist: public Extractor {
        unordered_map<int, int> lookup;

        struct Acc {
            Accumulator lines;   // number of lines in each shape
            Accumulator sizes;   // width/height of bb of each shape
            Accumulator areas;   // totoal area of bb of each shape
            Accumulator lengths; // total segment length of each shape

            static int constexpr DIMS =  1 + 2 * 4;

            double *extract (double *off) const {
                int c = ba::count(lines);
                if (c == 0) {
                    std::fill(off, off + DIMS, 0);
                    off += DIMS;
                }
                else {
                    *off++ = c;  // # lines
                    *off++ = ba::mean(lines);   // # lines
                    *off++ = std::sqrt(ba::variance(lines));  // # lines
                    *off++ = ba::mean(sizes);
                    *off++ = std::sqrt(ba::variance(sizes));
                    *off++ = ba::mean(areas);
                    *off++ = std::sqrt(ba::variance(areas));
                    *off++ = ba::mean(lengths);
                    *off++ = std::sqrt(ba::variance(lengths));
                }
                return off;
            }
        };

    public:
        shape_dist  (py::dict conf) {
            LOG(INFO) << "Creating shape_dist extractor";
            for (auto h: conf["shapes"].cast<py::list>()) {
                int code = h.cast<int>();
                lookup[code] = lookup.size();
            }
        }

        virtual size_t dim () const {
            return Acc::DIMS * (lookup.size() + 1);
        }

        virtual void apply (Layer const &l, double *ft) const {
            vector<Acc> accs(lookup.size()+1);
            double *ft0 = ft;
            for (auto const &shape: l.shapes) {
                int off = lookup.size();
                auto r = lookup.find(shape.code);
                if (r != lookup.end()) {
                    off = r->second;
                }
                Acc &acc = accs[off];
                Box b;
                double length = 0;
                acc.lines(shape.lines.size());
                for (auto const &l: shape.lines) {
                    b.extend(l[0]);
                    b.extend(l[1]);
                    length += norm(l);
                }
                acc.sizes(b.width());
                acc.sizes(b.height());
                acc.areas(b.area());
                acc.lengths(length);
            }
            for (auto const &acc: accs) {
                ft = acc.extract(ft);
            }
            CHECK(ft - ft0 == dim());
        }
    };

    class line_dist: public Extractor {
    public:
        line_dist (py::dict conf) {
            LOG(INFO) << "Creating line_dist extractor";
        }

        virtual size_t dim () const {
            return 0;
        }

        virtual void apply (Layer const &l, double *ft) const {
        }
    };

    //REGISTER(text_patterns);
    REGISTER(shape_dist);
    REGISTER(line_dist);
}}

