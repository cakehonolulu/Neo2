struct VSInput {
    float3 aPos : POSITION;
    uint aColorRaw : COLOR;
};

struct VSOutput {
    float4 position : SV_POSITION;
    float4 vertexColor : COLOR;
};

float4 unpackColor(uint color) {
    float r = float((color >> 24u) & 0xFFu) / 255.0;
    float g = float((color >> 16u) & 0xFFu) / 255.0;
    float b = float((color >> 8u) & 0xFFu) / 255.0;
    float a = float(color & 0xFFu) / 255.0;
    return float4(r, g, b, a);
}

VSOutput main(VSInput input) {
    VSOutput output;
    
    // Normalize pixel coordinates (0 -> 639 and 0 -> 223) to clip space (-1 to 1).
    float clipX = (input.aPos.x / 639.0) * 2.0 - 1.0;
    float clipY = 1.0 - (input.aPos.y / 223.0) * 2.0; // Flip Y if needed
    output.position = float4(clipX, clipY, input.aPos.z, 1.0);
    
    output.vertexColor = unpackColor(input.aColorRaw);
    return output;
}
