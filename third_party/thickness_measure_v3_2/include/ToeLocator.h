#ifndef TOE_LOCATOR_H_
#define TOE_LOCATOR_H_

#include <string>

#include "AppConfig.h"
#include "WeldMeasurementTypes.h"

namespace weld
{

class IToeLocator
{
public:
    virtual ~IToeLocator() {}
    virtual bool Locate(const PointCloudT::ConstPtr& sectionCloud, WeldToePair* toes, std::string* message) = 0;
};

class TraditionalToeLocator : public IToeLocator
{
public:
    virtual bool Locate(const PointCloudT::ConstPtr& sectionCloud, WeldToePair* toes, std::string* message);
};

class IntelligentToeLocator : public IToeLocator
{
public:
    virtual bool Locate(const PointCloudT::ConstPtr& sectionCloud, WeldToePair* toes, std::string* message);
};

class OnnxToeLocator : public IToeLocator
{
public:
    explicit OnnxToeLocator(const OnnxToeLocatorConfig& config);
    virtual ~OnnxToeLocator();

    virtual bool Locate(const PointCloudT::ConstPtr& sectionCloud, WeldToePair* toes, std::string* message);

private:
    struct Impl;
    Impl* impl_;
};

} // namespace weld

#endif // TOE_LOCATOR_H_
