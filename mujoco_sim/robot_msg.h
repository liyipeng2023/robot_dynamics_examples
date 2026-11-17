#pragma once
#include <cstdint>
#include <cstring>
#include <zmq.hpp>

#pragma pack(push, 1)
struct SimStateHeader {
    uint32_t magic;      // 0x52444C4D ("RDLM")
    uint32_t nq;
    uint32_t nv;
    uint32_t ncon;
    double   timestamp;
};

struct ContactItem {
    int32_t geom1;
    int32_t geom2;
    double  dist;            // 穿透深度 (m)
    double  pos[3];          // 接触点世界坐标
    double  normal[3];       // 接触法线方向
    double  force_local[6];  // 接触坐标系下 6D 力/力矩 [fn, ft1, ft2, tx, ty, tz]
    double  force_world[3];  // 世界坐标系下 3D 碰撞力
};

struct RdlCmdHeader {
    uint32_t magic;      // 0x52444C4D
    uint32_t mode;       // 0: 力矩控制模式, 1: 状态直写模式
    uint32_t nq;
    uint32_t nv;
};
#pragma pack(pop)

// 从 ZMQ 消息中零拷贝提取裸指针示例
inline bool parseSimStateZeroCopy(const zmq::message_t& msg,
                                  const SimStateHeader*& header_out,
                                  const double*& qpos_ptr,
                                  const double*& qvel_ptr,
                                  const double*& qfrc_constraint_ptr,
                                  const ContactItem*& contacts_ptr) {
    if (msg.size() < sizeof(SimStateHeader)) return false;
    const auto* raw = static_cast<const uint8_t*>(msg.data());
    header_out = reinterpret_cast<const SimStateHeader*>(raw);
    if (header_out->magic != 0x52444C4D) return false;

    size_t offset = sizeof(SimStateHeader);
    qpos_ptr = reinterpret_cast<const double*>(raw + offset);
    offset += header_out->nq * sizeof(double);

    qvel_ptr = reinterpret_cast<const double*>(raw + offset);
    offset += header_out->nv * sizeof(double);

    qfrc_constraint_ptr = reinterpret_cast<const double*>(raw + offset);
    offset += header_out->nv * sizeof(double);

    contacts_ptr = reinterpret_cast<const ContactItem*>(raw + offset);
    return true;
}