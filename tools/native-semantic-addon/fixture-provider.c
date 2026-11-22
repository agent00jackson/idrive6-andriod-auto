#include "fixture-provider.h"

enum { FIXTURE_REGION = 1 };

static const struct fixture_vertex vertices[] = {{0, 0}, {450, 0}, {450, 450}};
static const struct fixture_edge edges[] = {
    {FIXTURE_FIRST_LINK, 0, 1, 450}, {FIXTURE_SECOND_LINK, 1, 2, 450},
};

const struct fixture_graph basic_fixture_graph = {
    .vertices = vertices,
    .vertex_count = sizeof(vertices) / sizeof(vertices[0]),
    .edges = edges,
    .edge_count = sizeof(edges) / sizeof(edges[0]),
};

_Static_assert(sizeof(void *) == 4, "32-bit NavigationBasic ABI required");
_Static_assert(offsetof(struct basic_region_vector, begin) == 4, "region vector storage");
_Static_assert(offsetof(struct basic_region_vector, end) == 8, "region vector end");
_Static_assert(offsetof(struct basic_region_vector, allocation_end) == 12, "region vector capacity");
_Static_assert(offsetof(struct basic_fixture_map_root_methods, regions) == 0x20, "regions slot");
_Static_assert(offsetof(struct basic_fixture_map_root_methods, create_bmd) == 0x24, "BMD factory slot");
_Static_assert(offsetof(struct basic_fixture_map_root_methods, region_for_link) == 0x2c, "link region slot");
_Static_assert(offsetof(struct basic_fixture_bmd_methods, delete) == 0x04, "BMD deleting destructor");
_Static_assert(offsetof(struct basic_fixture_bmd_methods, optional_line_provider) == 0x34, "BMD line provider slot");
_Static_assert(offsetof(struct basic_fixture_routing_methods, split_link_id) == 0x28, "link decoding slot");

/* The tested call site supplies an empty vector. Its native destructor frees
 * the allocation, so the caller must supply the NavigationBasic allocator. */
static void fixture_regions(const struct basic_fixture_map_root *root, struct basic_region_vector *regions)
{
    if (regions->begin || regions->end || regions->allocation_end)
        return;
    uint16_t *value = root->memory.allocate(sizeof(*value));
    if (!value)
        return;
    *value = FIXTURE_REGION;
    regions->begin = value;
    regions->end = value + 1;
    regions->allocation_end = regions->end;
}

static void fixture_delete_bmd(struct basic_fixture_bmd *bmd)
{
    bmd->release(bmd);
}

static const void *fixture_optional_line_provider(const struct basic_fixture_bmd *bmd, uint32_t tile_level)
{
    (void)bmd;
    (void)tile_level;
    /* CRouteMapMatcher has a path geometry branch without geographic lines. */
    return NULL;
}

static const struct basic_fixture_bmd_methods fixture_bmd_methods = {
    .delete = fixture_delete_bmd,
    .optional_line_provider = fixture_optional_line_provider,
};

static struct basic_fixture_bmd *fixture_create_bmd(const struct basic_fixture_map_root *root, int16_t region)
{
    if (region != FIXTURE_REGION)
        return NULL;
    struct basic_fixture_bmd *bmd = root->memory.allocate(sizeof(*bmd));
    if (bmd) {
        bmd->methods = &fixture_bmd_methods;
        bmd->release = root->memory.release;
    }
    return bmd;
}

static int16_t fixture_region_for_link(const struct basic_fixture_map_root *root, uint64_t link)
{
    const uint64_t base_link = link & ~UINT64_C(1);
    for (size_t i = 0; i < root->graph->edge_count; ++i)
        if (base_link == root->graph->edges[i].id)
            return FIXTURE_REGION;
    return -1;
}

static const struct basic_fixture_map_root_methods fixture_methods = {
    .regions = fixture_regions,
    .create_bmd = fixture_create_bmd,
    .region_for_link = fixture_region_for_link,
};

void basic_fixture_map_root_init(struct basic_fixture_map_root *root, struct fixture_allocator memory)
{
    root->methods = &fixture_methods;
    root->graph = &basic_fixture_graph;
    root->memory = memory;
}

static uint64_t fixture_split_link_id(const struct basic_fixture_routing *routing,
                                     uint64_t directed_link, uint32_t *direction)
{
    (void)routing;
    *direction = directed_link & 1;
    return directed_link & ~UINT64_C(1);
}

static const struct basic_fixture_routing_methods fixture_routing_methods = {
    .split_link_id = fixture_split_link_id,
};

const struct basic_fixture_routing basic_fixture_routing = {.methods = &fixture_routing_methods};
