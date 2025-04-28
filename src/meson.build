glib2_deps = dependency('glib-2.0')
gthread2_deps = dependency('gthread-2.0')
gio2_deps = dependency('gio-2.0')
gio2_unix_deps = dependency('gio-unix-2.0')
libusb1_deps = dependency('libusb-1.0')
jsonc_deps = dependency('json-c')
gpiod_deps = dependency('libgpiod')
thread_deps = dependency('threads')

pcat_sources = [
    'main.c',
    'pmu-manager.c',
    'modem-manager.c',
    'controller.c'
]

pcat_headers = [
    'common.h',
    'pmu-manager.h',
    'modem-manager.h',
    'controller.h'
]

executable('pcat-manager',
    pcat_sources,
    pcat_headers,
    install: true,
    dependencies : [
        glib2_deps,
        gthread2_deps,
        gio2_deps,
        gio2_unix_deps,
        libusb1_deps,
        jsonc_deps,
        gpiod_deps,
        thread_deps
    ]
)
