#ifndef VIZDEFS_H
#define VIZDEFS_H

#define REGISTER_VIZUALIZATION(NAME)                                           \
  vizualization_t *NAME##_vizualization() {                                    \
    NAME##_vizualization_t *viz = malloc(sizeof(NAME##_vizualization_t));      \
    viz->viz.initialize = (void *)NAME##_initialize;                           \
    viz->viz.deinitialize = (void *)NAME##_deinitialize;                       \
    viz->viz.get_pixel_values = (void *)NAME##_get_pixel_values;               \
    viz->viz.tick = (void *)NAME##_tick;                                       \
    return (vizualization_t *)viz;                                             \
  }

#define REGISTER_VIZUALIZATION_WITH_CONFIG(NAME)                               \
  vizualization_t *NAME##_vizualization(NAME##_config_t config) {               \
    NAME##_vizualization_t *viz = malloc(sizeof(NAME##_vizualization_t));      \
    viz->viz.initialize = (void *)NAME##_initialize;                           \
    viz->viz.deinitialize = (void *)NAME##_deinitialize;                       \
    viz->viz.get_pixel_values = (void *)NAME##_get_pixel_values;               \
    viz->viz.tick = (void *)NAME##_tick;                                       \
    viz->config = config;                                                      \
    return (vizualization_t *)viz;                                             \
  }

#define DEFINE_VIZUALIZATION(NAME) vizualization_t *NAME##_vizualization();
#define DEFINE_VIZUALIZATION_WITH_CONFIG(NAME)                                 \
  vizualization_t *NAME##_vizualization(NAME##_config_t config);

#endif