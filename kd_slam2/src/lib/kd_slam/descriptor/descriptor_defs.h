#pragma once
#include "kd_slam/tree/tree_defs.h"
#include "kd_slam/tree/node_traits_.h"
#include "descriptor_.h"
#include "extractor_.h"
#include "matcher_.h"


namespace kd_slam { namespace descriptor {

// using Point2f=Eigen::Vector2f;
// using Point2fTraits=PointTraitsEigen_<Point2f>;
template <int Level_>
using ExtractorPoint2f_=Extractor_<NodePoint2f, Level_>;


template <int Level_>
using DescriptorPoint2f_=typename ExtractorPoint2f_<Level_>::DescriptorType;

template <int Level_>
using MatcherPoint2f_= Matcher_<DescriptorPoint2f_<Level_>>;


template <int Level_>
using ExtractorPoint3f_=Extractor_<NodePoint3f, Level_>;


template <int Level_>
using DescriptorPoint3f_=typename ExtractorPoint3f_<Level_>::DescriptorType;

template <int Level_>
using MatcherPoint3f_= Matcher_<DescriptorPoint3f_<Level_>>;

} } // namespace kd_slam::descriptor
