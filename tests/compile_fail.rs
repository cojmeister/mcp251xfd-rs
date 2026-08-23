#[test]
fn const_ram_overflow_fails_to_compile() {
    let t = trybuild::TestCases::new();
    t.compile_fail("tests/compile_fail/*.rs");
}
