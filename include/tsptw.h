
#ifndef VRP_TSPTW_H
#define VRP_TSPTW_H

#include <utility>

#include "model_data.h"

class tsp_label{
public:
    int         last_;
    double      deptime_;
    vector<bool>visited_;
    tsp_label*  next_;
    tsp_label(
            int             last,
            double          time,
            vector<bool>    visited
    ):
    last_(last),
    deptime_(time),
    visited_(std::move(visited))
    {
        next_ = nullptr;
    }

    ~tsp_label()= default;;

    bool allReachable(
        model_data*     modelData,
        vector<int>&    mapping,
        bool*           success,
        int             day
    );
};

class tsp_list{
public:
    tsp_label*      head_;
    int             length_;
    tsp_list()
    {
        head_ = nullptr;
        length_ = 0;
    }
    ~tsp_list()
    {
        if(length_ > 0)
        {
            tsp_label* lab;
            while (length_ > 0)
            {
                lab = extract_first();
                delete lab;
            }
        }
    }
    void insert_node(
            tsp_label*  label
    ){
        length_++;
        assert(label->next_ == nullptr);

        label->next_ = head_;
        head_ = label;
    };
    tsp_label* extract_first()
    {
        assert(length_ > 0);
        length_--;

        tsp_label* label;

        label = head_;
        head_ = head_->next_;

        return label;
    }

};


bool feasibleTSP(
    model_data*         modelData,
    std::vector<int>&   indexSet,
    int                 day
);

#endif //VRP_TSPTW_H
