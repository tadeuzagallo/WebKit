const array = new Array(1024);
array.fill(999.99);

function test(value, start, end) {
    array.fill(value, start, end);
}
noInline(test);

for (let i = 0; i < 1e4; i++) {
    test(20.12, 4, 824);
}
