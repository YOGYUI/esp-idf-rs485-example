#pragma once
#ifndef _SYSTEM_HPP_
#define _SYSTEM_HPP_

#include <vector>

#ifdef __cplusplus
extern "C" {
#endif

class CSystem
{
public:
    CSystem();
    virtual ~CSystem();
    static CSystem* Instance();

public:
    bool initialize();
    void release();

private:
    static CSystem* _instance;

    void print_system_info();
};

inline CSystem* GetSystem() {
    return CSystem::Instance();
}

#ifdef __cplusplus
};
#endif
#endif