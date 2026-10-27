#include "render/scene_pass.hpp"

#include <array>
#include <cstddef>
#include <stdexcept>

#include <sim_shaders.h>

#include "render/barrier.hpp"
#include "render/vulkan_context.hpp"

namespace sim::render {
namespace {

// Must match the push constant block in mesh.vert. 128 bytes, the minimum every
// device supports.
struct PushConstants {
  Mat4 view_projection;
  Mat4 model;
};
static_assert(sizeof(PushConstants) == 128);

vk::raii::ShaderModule create_shader_module(const vk::raii::Device& device, const unsigned char* code,
                                            std::size_t size) {
  // The embedded SPIR-V is 4-byte aligned, as Vulkan requires.
  return device.createShaderModule({
      .codeSize = size,
      .pCode = reinterpret_cast<const std::uint32_t*>(code),
  });
}

// The highest sample count not above the request that color and depth both support.
vk::SampleCountFlagBits choose_samples(const vk::raii::PhysicalDevice& physical_device, std::uint32_t requested) {
  const auto& limits = physical_device.getProperties().limits;
  const vk::SampleCountFlags supported = limits.framebufferColorSampleCounts & limits.framebufferDepthSampleCounts;
  for (const auto samples : {vk::SampleCountFlagBits::e8, vk::SampleCountFlagBits::e4, vk::SampleCountFlagBits::e2}) {
    if (static_cast<std::uint32_t>(samples) <= requested && (supported & samples)) {
      return samples;
    }
  }
  return vk::SampleCountFlagBits::e1;
}

}  // namespace

ScenePass::ScenePass(const VulkanContext& context, vk::Format color_format, vk::Extent2D extent,
                     std::uint32_t msaa_samples, std::span<const Mesh> meshes)
    : context_{context},
      color_format_{color_format},
      depth_format_{choose_depth_format(context.physical_device())},
      samples_{choose_samples(context.physical_device(), msaa_samples)} {
  const auto& device = context.device();
  resize(extent);
  set_meshes(meshes);

  const vk::PushConstantRange push_constant_range{
      .stageFlags = vk::ShaderStageFlagBits::eVertex,
      .offset = 0,
      .size = sizeof(PushConstants),
  };
  layout_ = device.createPipelineLayout({
      .pushConstantRangeCount = 1,
      .pPushConstantRanges = &push_constant_range,
  });

  // Modules are only needed while the pipeline is created.
  const vk::raii::ShaderModule vertex_shader =
      create_shader_module(device, sim_shaders_mesh_vert, sim_shaders_mesh_vert_size);
  const vk::raii::ShaderModule fragment_shader =
      create_shader_module(device, sim_shaders_mesh_frag, sim_shaders_mesh_frag_size);
  const std::array stages{
      vk::PipelineShaderStageCreateInfo{
          .stage = vk::ShaderStageFlagBits::eVertex,
          .module = *vertex_shader,
          .pName = "main",
      },
      vk::PipelineShaderStageCreateInfo{
          .stage = vk::ShaderStageFlagBits::eFragment,
          .module = *fragment_shader,
          .pName = "main",
      },
  };

  const vk::VertexInputBindingDescription binding{
      .binding = 0,
      .stride = sizeof(Vertex),
      .inputRate = vk::VertexInputRate::eVertex,
  };
  const std::array attributes{
      vk::VertexInputAttributeDescription{.location = 0,
                                          .binding = 0,
                                          .format = vk::Format::eR32G32B32Sfloat,
                                          .offset = offsetof(Vertex, position)},
      vk::VertexInputAttributeDescription{.location = 1,
                                          .binding = 0,
                                          .format = vk::Format::eR32G32B32Sfloat,
                                          .offset = offsetof(Vertex, normal)},
      vk::VertexInputAttributeDescription{.location = 2,
                                          .binding = 0,
                                          .format = vk::Format::eR32G32B32Sfloat,
                                          .offset = offsetof(Vertex, color)},
  };
  const vk::PipelineVertexInputStateCreateInfo vertex_input{
      .vertexBindingDescriptionCount = 1,
      .pVertexBindingDescriptions = &binding,
      .vertexAttributeDescriptionCount = static_cast<std::uint32_t>(attributes.size()),
      .pVertexAttributeDescriptions = attributes.data(),
  };
  const vk::PipelineInputAssemblyStateCreateInfo input_assembly{
      .topology = vk::PrimitiveTopology::eTriangleList,
  };
  // Viewport and scissor are dynamic, so resizing doesn't require a new pipeline.
  const vk::PipelineViewportStateCreateInfo viewport{.viewportCount = 1, .scissorCount = 1};
  const std::array dynamic_states{vk::DynamicState::eViewport, vk::DynamicState::eScissor};
  const vk::PipelineDynamicStateCreateInfo dynamic_state{
      .dynamicStateCount = static_cast<std::uint32_t>(dynamic_states.size()),
      .pDynamicStates = dynamic_states.data(),
  };
  const vk::PipelineRasterizationStateCreateInfo rasterization{
      .polygonMode = vk::PolygonMode::eFill,
      .cullMode = vk::CullModeFlagBits::eNone,
      .frontFace = vk::FrontFace::eCounterClockwise,
      .lineWidth = 1.0f,
  };
  const vk::PipelineMultisampleStateCreateInfo multisample{
      .rasterizationSamples = samples_,
  };
  const vk::PipelineDepthStencilStateCreateInfo depth_stencil{
      .depthTestEnable = vk::True,
      .depthWriteEnable = vk::True,
      .depthCompareOp = vk::CompareOp::eLess,
  };
  const vk::PipelineColorBlendAttachmentState blend_attachment{
      .colorWriteMask = vk::ColorComponentFlagBits::eR | vk::ColorComponentFlagBits::eG |
                        vk::ColorComponentFlagBits::eB | vk::ColorComponentFlagBits::eA,
  };
  const vk::PipelineColorBlendStateCreateInfo color_blend{
      .attachmentCount = 1,
      .pAttachments = &blend_attachment,
  };
  // With dynamic rendering, the pipeline declares its attachment formats directly.
  const vk::PipelineRenderingCreateInfo rendering{
      .colorAttachmentCount = 1,
      .pColorAttachmentFormats = &color_format_,
      .depthAttachmentFormat = depth_format_,
  };

  pipeline_ = device.createGraphicsPipeline(nullptr, {
      .pNext = &rendering,
      .stageCount = static_cast<std::uint32_t>(stages.size()),
      .pStages = stages.data(),
      .pVertexInputState = &vertex_input,
      .pInputAssemblyState = &input_assembly,
      .pViewportState = &viewport,
      .pRasterizationState = &rasterization,
      .pMultisampleState = &multisample,
      .pDepthStencilState = &depth_stencil,
      .pColorBlendState = &color_blend,
      .pDynamicState = &dynamic_state,
      .layout = *layout_,
  });
}

void ScenePass::set_meshes(std::span<const Mesh> meshes) {
  // All meshes share one vertex buffer.
  std::vector<Vertex> vertices;
  meshes_.clear();
  for (const Mesh& mesh : meshes) {
    meshes_.push_back({
        .first_vertex = static_cast<std::uint32_t>(vertices.size()),
        .vertex_count = static_cast<std::uint32_t>(mesh.vertices.size()),
    });
    vertices.insert(vertices.end(), mesh.vertices.begin(), mesh.vertices.end());
  }
  if (vertices.empty()) {
    throw std::invalid_argument{"the scene needs at least one vertex"};
  }
  const auto vertex_bytes = std::as_bytes(std::span{vertices});
  vertex_buffer_ = create_host_buffer(context_, vk::BufferUsageFlagBits::eVertexBuffer, vertex_bytes.size());
  write_host_buffer(vertex_buffer_, vertex_bytes);
}

void ScenePass::resize(vk::Extent2D extent) {
  extent_ = extent;
  depth_image_ = create_attachment_image(context_, depth_format_, extent_, samples_,
                                         vk::ImageUsageFlagBits::eDepthStencilAttachment,
                                         vk::ImageAspectFlagBits::eDepth);
  if (multisampled()) {
    // Transient: its contents only live within a frame, before being resolved.
    color_image_ = create_attachment_image(
        context_, color_format_, extent_, samples_,
        vk::ImageUsageFlagBits::eColorAttachment | vk::ImageUsageFlagBits::eTransientAttachment,
        vk::ImageAspectFlagBits::eColor);
  }
}

void ScenePass::record(const vk::raii::CommandBuffer& command_buffer, vk::ImageView output,
                       const FrameView& view, vk::Buffer world_vertex_buffer) const {
  // The attachments are cleared every frame, so their old contents can be discarded.
  // The source scopes wait for the previous frame's writes to them.
  if (multisampled()) {
    transition(command_buffer, *color_image_.image, vk::ImageAspectFlagBits::eColor,
               vk::ImageLayout::eUndefined, vk::ImageLayout::eColorAttachmentOptimal,
               vk::PipelineStageFlagBits2::eColorAttachmentOutput,
               vk::AccessFlagBits2::eColorAttachmentWrite,
               vk::PipelineStageFlagBits2::eColorAttachmentOutput,
               vk::AccessFlagBits2::eColorAttachmentWrite);
  }
  constexpr auto depth_stages = vk::PipelineStageFlagBits2::eEarlyFragmentTests |
                                vk::PipelineStageFlagBits2::eLateFragmentTests;
  transition(command_buffer, *depth_image_.image, vk::ImageAspectFlagBits::eDepth,
             vk::ImageLayout::eUndefined, vk::ImageLayout::eDepthAttachmentOptimal, depth_stages,
             vk::AccessFlagBits2::eDepthStencilAttachmentWrite, depth_stages,
             vk::AccessFlagBits2::eDepthStencilAttachmentRead |
                 vk::AccessFlagBits2::eDepthStencilAttachmentWrite);

  // With multisampling, render into the multisampled image and resolve (average the
  // samples) into the output at the end; the samples themselves aren't kept.
  const vk::RenderingAttachmentInfo color_attachment{
      .imageView = multisampled() ? *color_image_.view : output,
      .imageLayout = vk::ImageLayout::eColorAttachmentOptimal,
      .resolveMode = multisampled() ? vk::ResolveModeFlagBits::eAverage : vk::ResolveModeFlagBits::eNone,
      .resolveImageView = multisampled() ? output : vk::ImageView{},
      .resolveImageLayout = vk::ImageLayout::eColorAttachmentOptimal,
      .loadOp = vk::AttachmentLoadOp::eClear,
      .storeOp = multisampled() ? vk::AttachmentStoreOp::eDontCare : vk::AttachmentStoreOp::eStore,
      .clearValue = vk::ClearValue{.color = vk::ClearColorValue{.float32 = view.clear_color}},
  };
  const vk::RenderingAttachmentInfo depth_attachment{
      .imageView = *depth_image_.view,
      .imageLayout = vk::ImageLayout::eDepthAttachmentOptimal,
      .loadOp = vk::AttachmentLoadOp::eClear,
      .storeOp = vk::AttachmentStoreOp::eDontCare,
      .clearValue = vk::ClearValue{.depthStencil = {.depth = 1.0f, .stencil = 0}},
  };
  command_buffer.beginRendering({
      .renderArea = {.offset = {.x = 0, .y = 0}, .extent = extent_},
      .layerCount = 1,
      .colorAttachmentCount = 1,
      .pColorAttachments = &color_attachment,
      .pDepthAttachment = &depth_attachment,
  });
  draw(command_buffer, view, world_vertex_buffer);
  command_buffer.endRendering();
}

void ScenePass::draw(const vk::raii::CommandBuffer& command_buffer, const FrameView& view,
                     vk::Buffer world_vertex_buffer) const {
  command_buffer.bindPipeline(vk::PipelineBindPoint::eGraphics, *pipeline_);
  command_buffer.setViewport(0, vk::Viewport{
      .x = 0.0f,
      .y = 0.0f,
      .width = static_cast<float>(extent_.width),
      .height = static_cast<float>(extent_.height),
      .minDepth = 0.0f,
      .maxDepth = 1.0f,
  });
  command_buffer.setScissor(0, vk::Rect2D{.offset = {.x = 0, .y = 0}, .extent = extent_});
  command_buffer.bindVertexBuffers(0, {*vertex_buffer_.buffer}, {vk::DeviceSize{0}});

  for (const DrawItem& item : view.items) {
    const MeshRange& mesh = meshes_.at(item.mesh);
    command_buffer.pushConstants<PushConstants>(
        *layout_, vk::ShaderStageFlagBits::eVertex, 0,
        PushConstants{.view_projection = view.view_projection, .model = item.model});
    command_buffer.draw(mesh.vertex_count, 1, mesh.first_vertex, 0);
  }

  if (!view.world_vertices.empty()) {
    constexpr Mat4 identity{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
    command_buffer.bindVertexBuffers(0, {world_vertex_buffer}, {vk::DeviceSize{0}});
    command_buffer.pushConstants<PushConstants>(
        *layout_, vk::ShaderStageFlagBits::eVertex, 0,
        PushConstants{.view_projection = view.view_projection, .model = identity});
    command_buffer.draw(static_cast<std::uint32_t>(view.world_vertices.size()), 1, 0, 0);
  }
}

}  // namespace sim::render
