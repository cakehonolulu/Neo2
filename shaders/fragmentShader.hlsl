struct PSInput {
    float4 vertexColor : COLOR;
};

float4 main(PSInput input) : SV_TARGET {
    return input.vertexColor;
}
