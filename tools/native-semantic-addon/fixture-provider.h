#ifndef BASIC_FIXTURE_PROVIDER_H
#define BASIC_FIXTURE_PROVIDER_H

#include <stddef.h>
#include <stdint.h>

struct fixture_vertex { int32_t east_dm, north_dm; };
struct fixture_edge { uint32_t id, from, to, length_dm; };

struct fixture_graph {
    const struct fixture_vertex *vertices;
    size_t vertex_count;
    const struct fixture_edge *edges;
    size_t edge_count;
};

extern const struct fixture_graph basic_fixture_graph;

struct fixture_allocator {
    void *(*allocate)(size_t);
    void (*release)(void *);
};

#define FIXTURE_FIRST_LINK UINT64_C(2)
#define FIXTURE_SECOND_LINK UINT64_C(4)

struct basic_region_vector {
    uint32_t allocator;
    uint16_t *begin;
    uint16_t *end;
    uint16_t *allocation_end;
};

struct basic_fixture_bmd;
struct basic_fixture_bmd_methods {
    void (*unverified_destructor)(void);
    void (*delete)(struct basic_fixture_bmd *);
    void (*unverified_methods[11])(void);
    const void *(*optional_line_provider)(const struct basic_fixture_bmd *, uint32_t);
};

struct basic_fixture_bmd {
    const struct basic_fixture_bmd_methods *methods;
    void (*release)(void *);
};

struct basic_fixture_map_root;
struct basic_fixture_map_root_methods {
    void (*unverified_prefix[8])(void);
    void (*regions)(const struct basic_fixture_map_root *, struct basic_region_vector *);
    struct basic_fixture_bmd *(*create_bmd)(const struct basic_fixture_map_root *, int16_t);
    void (*unverified_intermediate)(void);
    int16_t (*region_for_link)(const struct basic_fixture_map_root *, uint64_t);
};

/* Bounded accessors for isolated tests. Production vtables have additional slots. */
struct basic_fixture_map_root {
    const struct basic_fixture_map_root_methods *methods;
    const struct fixture_graph *graph;
    struct fixture_allocator memory;
};

void basic_fixture_map_root_init(struct basic_fixture_map_root *, struct fixture_allocator);

struct basic_fixture_routing;
struct basic_fixture_routing_methods {
    void (*unverified_prefix[10])(void);
    uint64_t (*split_link_id)(const struct basic_fixture_routing *, uint64_t, uint32_t *);
};

struct basic_fixture_routing {
    const struct basic_fixture_routing_methods *methods;
};

extern const struct basic_fixture_routing basic_fixture_routing;

#endif
